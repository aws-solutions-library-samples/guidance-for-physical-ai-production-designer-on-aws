//! factoryos-backend: the Factory OS viewer's single entry point.
//!
//! One public port serves:
//! * `/api/v1/…`: config and subsystem control (HTTP or HTTPS)
//! * `/zenoh`: Zenoh over WebSocket (WS or WSS) for wasm viewers, via the remote-api plugin
//! * `/zenoh-native`: native viewers' Zenoh sessions over WebSocket (`--zenoh-ws-port`)
//! * `/rtc/…`: WebRTC streams (Isaac Sim's web viewer), through a loopback relay ([`services::rtc`])
//! * `/smartpad/…`: the cell's KUKA smartPADs, through each controller's KDM ([`services::smartpad`]; the edge maps
//!   each one's own hostname here)
//! * `/dcv/…`: the Windows IDE's NICE DCV, through a DCV Connection Gateway on this machine
//!   ([`services::dcv`]; with its session resolver and token verifier on their own listeners)
//! * `/`: the wasm viewer itself (`--web-root`)
//!
//! All of it also under any workspace's prefix, `/workspace/<name>/…` (see [`workspace_prefix`]).
//! Native viewers may instead connect to the Zenoh router directly (`--zenoh-listen`).

mod activity;
mod authz;
mod bus;
mod config_store;
/// The internal deployment's defaults; not in the public export (every use has a fallback without it).
#[cfg(feature = "internal")]
mod internal;
mod origin_verify;
mod paths;
mod chat_ws;
mod services;
mod storage;
mod workspace_prefix;
mod ws_proxy;

use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::Arc;

use anyhow::Context as _;
use axum::extract::State;
use axum::http::StatusCode;
use axum::routing::{get, put};
use axum::{Json, Router};
use clap::Parser;
use factoryos_protocol::config::{Config, ConfigResponse, Secrets};
use factoryos_protocol::storage::Scope;
use factoryos_protocol::{Health, PROTOCOL_VERSION, api, keys};
use tower_http::cors::CorsLayer;
use tower_http::services::{ServeDir, ServeFile};
use tower_http::trace::TraceLayer;

use crate::bus::{Bus, BusOptions};
use crate::config_store::ConfigStore;
use crate::services::agentic::AgenticService;
use crate::services::cell::CellService;
use crate::services::codegen::CodegenService;
use crate::services::dresser::DresserService;
use crate::services::fitter::FitterService;
use crate::services::hardware::HardwareService;
use crate::services::infra::InfraService;
use crate::services::product::ProductService;

#[derive(Parser)]
#[command(version, about)]
struct Args {
    /// HTTP(S) + WS(S) address. Loopback unless asked for (e.g. `0.0.0.0:80` behind the workspace edge).
    #[arg(long, env = "FACTORY_OS_BIND", default_value = "127.0.0.1:8080")]
    bind: SocketAddr,

    /// PEM certificate chain. Together with `--tls-key`, serves HTTPS/WSS instead of HTTP/WS.
    #[arg(long, env = "FACTORY_OS_TLS_CERT", requires = "tls_key")]
    tls_cert: Option<PathBuf>,
    /// PEM private key.
    #[arg(long, env = "FACTORY_OS_TLS_KEY", requires = "tls_cert")]
    tls_key: Option<PathBuf>,

    /// Zenoh endpoints for native viewers and other peers. Repeat for several; `none` disables.
    #[arg(long, env = "FACTORY_OS_ZENOH_LISTEN", default_value = "tcp/127.0.0.1:7447", value_delimiter = ',')]
    zenoh_listen: Vec<String>,

    /// Loopback port of the Zenoh remote-api plugin, which `/zenoh` proxies to.
    #[arg(long, default_value_t = 10000)]
    remote_api_port: u16,

    /// Loopback port of the router's Zenoh WebSocket listener, which `/zenoh-native` proxies to
    /// (native viewers that can only reach the public port). `0` disables.
    #[arg(long, default_value_t = 10001)]
    zenoh_ws_port: u16,

    /// Config file, created with defaults if missing. Default: `factoryos-viewer/factoryos.config.json`.
    #[arg(long, env = "FACTORY_OS_CONFIG")]
    config: Option<PathBuf>,

    /// Directory with the built wasm viewer (index.html, *.wasm), served at `/`.
    #[arg(long, env = "FACTORY_OS_WEB_ROOT")]
    web_root: Option<PathBuf>,

    /// The workspace this backend serves: other workspaces' `/workspace/<name>/…` are then 404.
    /// Without it, any workspace's prefix reaches this backend.
    #[arg(long, env = "FACTORY_OS_WORKSPACE", value_parser = parse_workspace)]
    workspace: Option<String>,

    /// The release (CI build id) this backend was deployed from, reported in `/api/v1/health`.
    #[arg(long, env = "FACTORY_OS_RELEASE")]
    release: Option<String>,

    /// Workspace IDE roots besides the active cell's folder: `label=/absolute/path`; repeat, or
    /// `;`-separated in the variable.
    #[arg(long = "ide-root", env = "FACTORY_OS_IDE_ROOTS", value_delimiter = ';')]
    ide_roots: Vec<String>,

    /// Where the user's files are kept (src/storage): `local` (the backend's folders, the default) or
    /// `s3` (`--s3-bucket`, `users/<--workspace>/…` and the shared `catalogue/…`; the folders stay
    /// as the working copy).
    #[arg(long, env = "FACTORY_OS_STORAGE")]
    storage: Option<String>,
    /// The bucket of `--storage s3`.
    #[arg(long, env = "FACTORY_OS_S3_BUCKET")]
    s3_bucket: Option<String>,
    /// Its region; else `AWS_REGION` / `AWS_DEFAULT_REGION`, else the instance's.
    #[arg(long, env = "FACTORY_OS_S3_REGION")]
    s3_region: Option<String>,
    /// A prefix for everything in the bucket (normally none).
    #[arg(long, env = "FACTORY_OS_S3_ROOT")]
    s3_root: Option<String>,
    /// Another S3 endpoint, path-style (HTTPS, or HTTP on loopback: a local MinIO).
    #[arg(long, env = "FACTORY_OS_S3_ENDPOINT")]
    s3_endpoint: Option<String>,

    /// Who may write and organise the shared catalogue (src/authz.rs): `all`, `none`, or
    /// `alias,alias,…`. Unset: the local user only (a loopback caller without the workspace edge).
    #[arg(long, env = "FACTORY_OS_CATALOGUE_ADMINS")]
    catalogue_admins: Option<String>,

    /// The workspace edge serves the bucket at `/_data` (`--storage s3` behind CloudFront): the
    /// owner's viewer reads and writes through it instead of presigned URLs (storage::http).
    /// `FACTORY_OS_EDGE_DATA=1|true|yes|on` (what `factoryos-workspace configure` writes); a plain
    /// `bool` env flag would reject `1` and keep the backend from starting.
    #[arg(long, env = "FACTORY_OS_EDGE_DATA", num_args = 0..=1, default_missing_value = "true",
          value_parser = clap::builder::BoolishValueParser::new(), default_value = "false")]
    edge_data: bool,
}

fn parse_workspace(name: &str) -> Result<String, String> {
    if factoryos_protocol::workspace::is_valid_name(name) {
        Ok(name.to_owned())
    } else {
        Err("a workspace name is [A-Za-z0-9._-]+".to_owned())
    }
}

/// How often the backend pings its WebSockets (chat, IDE terminals, terminal agents' links): an idle one
/// is closed on the way (CloudFront, the ALB) after a few minutes, so a quiet chat or shell would drop.
pub const WS_KEEPALIVE: std::time::Duration = std::time::Duration::from_secs(30);

#[derive(Clone)]
pub struct AppState {
    pub config: Arc<ConfigStore>,
    pub bus: Bus,
    pub cell: Arc<CellService>,
    pub dresser: Arc<DresserService>,
    pub fitter: Arc<FitterService>,
    pub training: Arc<services::training::TrainingService>,
    pub product: Arc<ProductService>,
    pub agentic: Arc<AgenticService>,
    pub codegen: Arc<CodegenService>,
    pub hardware: Arc<HardwareService>,
    pub infra: Arc<InfraService>,
    pub chat: Arc<services::chat::ChatService>,
    pub remote_api: SocketAddr,
    /// The router's loopback `ws` listener, for [`ws_proxy::native_handler`].
    pub zenoh_ws: Option<SocketAddr>,
    pub health: Arc<Health>,
    pub activity: Arc<activity::Activity>,
    pub agent_login: Arc<services::agent_login::AgentLoginService>,
    /// The workspace files' admission ([`services::files::admit_transfer`]), for Product & Task's uploads.
    pub files: Arc<services::files::FilesService>,
    /// The files 3D scenes reference, served by content hash ([`api::ASSET`]).
    pub assets: Arc<services::assets::AssetIndex>,
    /// Where the user's files are kept: written through to after each save ([`storage`]).
    pub storage: Arc<storage::Storage>,
}

impl axum::extract::FromRef<AppState> for Arc<services::assets::AssetIndex> {
    fn from_ref(state: &AppState) -> Self {
        state.assets.clone()
    }
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    // `fos-migrate-cells …` (a link to this binary) or `factoryos-backend migrate-cells …`.
    let argv: Vec<String> = std::env::args().collect();
    let invoked = argv.first().map(|a| std::path::Path::new(a).file_name().unwrap_or_default().to_string_lossy().into_owned());
    let migrate_args = match (invoked.as_deref(), argv.get(1).map(String::as_str)) {
        (Some("fos-migrate-cells"), _) => Some(&argv[1..]),
        (_, Some("migrate-cells")) => Some(&argv[2..]),
        _ => None,
    };
    if let Some(args) = migrate_args {
        tracing_subscriber::fmt().with_env_filter("warn").with_writer(std::io::stderr).init();
        let outcome = tokio::task::spawn_blocking({
            let args = args.to_vec();
            move || services::cell_migrate::main(&args)
        })
        .await??;
        if !outcome.touched.is_empty() {
            // The store follows (S3 mode: the backend's settings, from its environment).
            if let Err(err) = migrated_cells_to_store(&argv[0], &outcome).await {
                eprintln!("The migrated cells are NOT in the store yet ({err:#}): run `factoryos-backend migrate-to-s3`");
            }
        }
        std::process::exit(if outcome.ok { 0 } else { 1 });
    }
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "info,zenoh=warn,zenoh_plugin_remote_api=warn".into()),
        )
        .init();
    // `factoryos-backend migrate-to-s3 [the usual options]`: upload this workspace's products, cells
    // and dressed robots into its S3 subtree (keys already there are skipped), then exit.
    if argv.get(1).map(String::as_str) == Some("migrate-to-s3") {
        let args = Args::parse_from(argv[..1].iter().chain(&argv[2..]));
        return migrate_to_s3(args).await;
    }
    let args = Args::parse();
    // The public deployment's origin check (deployment/CONTRACT.md): read first, so a malformed value stops here.
    let origin_secret = origin_verify::OriginSecret::from_env()?;
    if origin_secret.is_some() {
        tracing::info!("Origin check: requests from peers other than loopback need {}", origin_verify::HEADER);
    }

    let zenoh_listen: Vec<String> = args.zenoh_listen.into_iter().filter(|e| e != "none").collect();
    let remote_api = SocketAddr::from(([127, 0, 0, 1], args.remote_api_port));
    let zenoh_ws = (args.zenoh_ws_port != 0).then(|| SocketAddr::from(([127, 0, 0, 1], args.zenoh_ws_port)));
    // The `ws` listener is reached through the public port, so it isn't advertised itself.
    let listen = zenoh_listen.iter().cloned().chain(zenoh_ws.map(|addr| format!("ws/{addr}"))).collect();
    let bus = Bus::start(&BusOptions { listen, remote_api })
    .await?;

    let config = Arc::new(ConfigStore::open(args.config.unwrap_or_else(paths::default_config_file)));
    tracing::info!("Config: {}", config.response().file);
    // The active reference cell runs from its view (see services::cell_resolve).
    if let Some(changed) = services::cells::normalise_active(&config) {
        tracing::info!("Active cell: now runs from {}", changed.cell_dir);
    }
    let storage_options = storage::StorageOptions {
        mode: args.storage.clone(),
        bucket: args.s3_bucket.clone(),
        region: args.s3_region.clone(),
        root: args.s3_root.clone(),
        endpoint: args.s3_endpoint.clone(),
    };
    let storage = storage::Storage::open(&storage_options, args.workspace.as_deref(), {
        let config = config.clone();
        move || storage::local::Layout::of_config(&config)
    })
    .await?;
    // S3 mode: the user's settings live in the store (users/<owner>/factoryos.config.json), the
    // machine's in the file; the store's win (config_store). Read again every 20 s.
    match config.attach_storage(storage.clone()).await {
        Ok(true) => tracing::info!("Config: your settings from the store"),
        Ok(false) => {}
        Err(err) => tracing::warn!("Config: your settings could not be read from the store ({err:#}); saves fail until they are"),
    }
    if storage.mode() == factoryos_protocol::storage::StorageMode::S3 {
        let (config, bus) = (config.clone(), bus.clone());
        tokio::spawn(async move {
            loop {
                tokio::time::sleep(config_store::REFRESH_EVERY).await;
                match config.refresh().await {
                    Ok(true) => bus.put_json(keys::CONFIG, &config.get()),
                    Ok(false) => {}
                    Err(err) => tracing::warn!("Config: reading your settings from the store: {err:#}"),
                }
            }
        });
    }
    // The services' writes of user files reach it (S3 mode; storage::saved and the others), for as
    // long as the backend runs.
    let _write_through = storage::register(&storage);
    // Scenes moved from the catalogue's fitted-robots/ to scenes/: before anything reads them, and
    // written through (S3 mode) as any change of the user's files.
    services::scene_migrate::migrate_config(&config);
    services::scene_migrate::run_local(&config);
    // The shared catalogue's working copy follows `catalogue/` (S3 mode), and every change is told.
    let catalogue_sync = storage::sync::CatalogueSync::new(storage.clone(), services::assets::default_state_dir());
    // At startup (S3 mode): the first time, what this workspace has goes up; then what the user's
    // subtree has and the working copy doesn't yet comes down; then the shared catalogue's sync runs.
    tokio::spawn({
        let (storage, catalogue_sync, config) = (storage.clone(), catalogue_sync.clone(), config.clone());
        let data_dir = std::path::Path::new(&config.response().file).parent().map(std::path::Path::to_owned);
        async move {
            if let Some(dir) = data_dir {
                match storage::migrate::migrate_once(&storage, &dir).await {
                    Ok(Some(summary)) => tracing::info!("Storage: migrate-to-s3: {summary}"),
                    Ok(None) => {}
                    Err(err) => tracing::warn!("Storage: migrate-to-s3: {err:#}"),
                }
            }
            // The user's own catalogue additions too, before the shared catalogue's sync (which never
            // overwrites them). The same prefixes the migration uploads.
            for prefix in storage::USER_PREFIXES {
                match storage.hydrate(Scope::User, prefix).await {
                    Ok(0) => {}
                    Ok(n) => tracing::info!("Storage: {n} file(s) of {prefix} from the store"),
                    Err(err) => tracing::warn!("Storage: hydrating {prefix}: {err:#}"),
                }
            }
            // The user's scene keys of the old folder (S3).
            match services::scene_migrate::migrate_store(&storage).await {
                Ok(0) => {}
                Ok(n) => tracing::info!("Scenes: {n} scene key(s) moved from catalogue/fitted-robots/ to catalogue/scenes/"),
                Err(err) => tracing::warn!("Scenes: moving the store's scene keys: {err:#}"),
            }
            // The user's scenes that came down flat (`scenes/<f>.yaml`) become scene folders, written
            // through as above.
            services::scene_migrate::run_local(&config);
            catalogue_sync.spawn();
            // What other processes write into the cells (the planner's plans, the auto-restart
            // setting): written through once it is there; first after the fill above.
            storage::watch::Watch::new(storage.clone()).spawn({
                move || services::cells::cells_folder(&config).map(|dir| storage::watch::cell_outputs(&dir)).unwrap_or_default()
            });
        }
    });

    // The simulated cell-runner (step 1 Load & Play), on `factoryos/sim/*`; `factoryos/cell/*` is
    // the live cell's (fos-live-bridge / fos-live-scene on a server connected to it).
    let cell = CellService::new(bus.clone());
    let shutdown_cell = cell.clone();
    tokio::spawn({
        let cell = cell.clone();
        async move {
            if let Err(err) = cell.serve_queries().await {
                tracing::error!("cell queryables stopped: {err:#}");
            }
        }
    });
    {
        let config = config.clone();
        bus::serve_latest(&bus, keys::CONFIG, move || Some(config.get()));
    }
    // Producers outside the backend (fos-live-scene) re-register their assets when it changes.
    let boot_id = bus::boot_id();
    bus::announce_boot(&bus, boot_id.clone());

    let agentic = AgenticService::new(&bus, config.clone());
    let shutdown_agentic = agentic.clone();
    // Control Code Export: the plan2cell CLI.
    let codegen = CodegenService::new(&bus, config.clone());
    // Hardware Configuration (plan2cell's addressing, saved per cell) and the Virtual Cell (the
    // cell-control Lambda).
    let hardware = HardwareService::new(config.clone());
    let infra = InfraService::new(&bus, config.clone(), hardware.clone());
    // The planner chat (kiro-cli over ACP); its process only starts with the first prompt.
    // One session per chat tab, each its own process; their logs next to the config file.
    let chat_dir = std::path::Path::new(&config.response().file)
        .parent()
        .map_or_else(|| PathBuf::from("."), std::path::Path::to_owned)
        .join("chat-sessions");
    let chat = services::chat::ChatService::new(config.clone(), services::chat::ChatOptions::from_env(chat_dir));
    let shutdown_chat = chat.clone();
    let assets = services::assets::AssetIndex::new({
        let config = config.clone();
        move || services::asset_register::allowed_roots(&config)
    });
    services::asset_register::serve(&bus, config.clone(), assets.clone());
    let shutdown_assets = assets.clone();
    let dresser = DresserService::new(&bus, config.clone(), assets.clone());
    // Robots dressed before the Robo Dresser wrote TCP frames get the one their tool declares.
    tokio::task::spawn_blocking({
        let (config, bus) = (config.clone(), bus.clone());
        move || {
            let catalogue = PathBuf::from(config.resolve(&config.get().catalogue));
            let release = PathBuf::from(paths::default_catalogue());
            let cells = services::cells::cells_folder(&config);
            for change in services::tcp::upgrade_tcp_frames(&catalogue, &release, cells.as_deref()) {
                tracing::info!("TCP upgrade: {change}");
                bus.log(format!("TCP upgrade: {change}"));
            }
        }
    });
    let fitter = FitterService::new(&bus, config.clone(), assets.clone());
    let product = ProductService::new(&bus, config.clone(), assets.clone());
    {
        // Saved Robo Fitter scenes go to the store (S3 mode).
        let storage = storage.clone();
        fitter.set_on_saved(move |scene| storage.spawn_save_path(scene.to_owned()));
    }
    {
        // Newly dressed robots show up in the Robo Fitter's palette.
        // And the dressed pair goes to the user's own catalogue additions in the store.
        let (fitter, storage, bus) = (fitter.clone(), storage.clone(), bus.clone());
        dresser.set_on_dressed(move |dressed| {
            fitter.refresh_palette();
            storage.spawn_save_tree(dressed.to_owned());
            // The viewers list the catalogue again (its working copy changed here).
            bus.put_json(keys::CATALOGUE_CHANGED, &factoryos_protocol::storage::CatalogueChanged::default());
        });
    }
    {
        // A change of the shared catalogue: the viewers list again (bus), and so do the Robo Fitter's
        // palette and the Robo Dresser's catalogue here.
        let (bus, fitter, dresser) = (bus.clone(), fitter.clone(), dresser.clone());
        catalogue_sync.on_change(move |changed| {
            bus.put_json(keys::CATALOGUE_CHANGED, changed);
            fitter.refresh_palette();
            dresser.open(true);
        });
    }
    tokio::spawn({
        let dresser = dresser.clone();
        async move {
            if let Err(err) = dresser.serve_joints().await {
                tracing::error!("dresser joint updates stopped: {err:#}");
            }
        }
    });
    tokio::spawn({
        let fitter = fitter.clone();
        async move {
            if let Err(err) = fitter.serve_edits().await {
                tracing::error!("fitter scene edits stopped: {err:#}");
            }
        }
    });
    // Linked robots follow the simulation (`factoryos/sim/state`), never the live cell.
    tokio::spawn({
        let fitter = fitter.clone();
        async move {
            if let Err(err) = fitter.follow_cell().await {
                tracing::error!("fitter cell following stopped: {err:#}");
            }
        }
    });

    let training = services::training::TrainingService::new(&bus, config.clone(), fitter.reach_context(), assets.clone());
    // The roles of a caller (catalogue admins), one seam for every check (src/authz.rs).
    let authz = authz::ConfigAuthz::new(args.catalogue_admins.as_deref(), args.workspace.clone())?;
    tracing::info!("Catalogue admins: {}", authz.describe());
    let authz: Arc<dyn authz::Authz> = authz;
    let files = services::files::FilesService::new(config.clone(), &args.ide_roots, args.workspace.clone())?;
    files.set_authz(authz.clone());
    let alive_bus = bus.clone();
    let state = AppState {
        config,
        bus,
        cell,
        dresser,
        fitter,
        training,
        product,
        agentic,
        codegen,
        hardware,
        infra,
        chat,
        remote_api,
        zenoh_ws,
        health: Arc::new(Health {
            protocol_version: PROTOCOL_VERSION,
            backend_version: env!("CARGO_PKG_VERSION").to_owned(),
            zenoh_endpoints: zenoh_listen,
            zenoh_ws: zenoh_ws.is_some(),
            workspace: args.workspace.clone(),
            release: args.release.clone().filter(|release| !release.is_empty()),
            idle_secs: None,
            open_connections: None,
            boot_id: boot_id.clone(),
        }),
        activity: activity::Activity::new(),
        agent_login: services::agent_login::AgentLoginService::new(),
        files: files.clone(),
        assets,
        storage: storage.clone(),
    };
    let activity = state.activity.clone();
    let storage_assets = state.assets.clone();
    // NICE DCV at /dcv/ (a DCV Connection Gateway on this machine): the Windows IDE from the cell's status.
    let dcv = {
        let infra = state.infra.clone();
        let ide: services::dcv::IdeLookup = Arc::new(move || {
            let infra = infra.clone();
            Box::pin(async move { infra.windows_ide_address().await })
        });
        // The workspace's owner is the edge user who may use it (X-User-Alias).
        let settings = services::dcv::Settings { owner: args.workspace.clone(), ..services::dcv::Settings::from_env() };
        let dcv = services::dcv::DcvService::new(settings, ide, activity.clone());
        // Open PLC IDE: when its window connects, CODESYS comes to the front there with the head project.
        let infra = state.infra.clone();
        dcv.on_open(Arc::new(move || {
            let infra = infra.clone();
            tokio::spawn(async move {
                match infra.show_ide().await {
                    Ok(answer) => tracing::info!("Open PLC IDE: CODESYS to the front: {answer}"),
                    Err(err) => tracing::warn!("Open PLC IDE: could not bring CODESYS to the front: {err:#}"),
                }
            });
        }));
        dcv
    };
    state.infra.set_dcv_gateway(dcv.enabled(), dcv.has_fallback_server(), dcv.target_names());
    dcv.start();
    // WebRTC streams at /rtc/ (Isaac Sim's web viewer through a loopback relay), the owner's as DCV.
    let rtc = services::rtc::RtcService::new(
        services::rtc::Settings { owner: args.workspace.clone(), ..services::rtc::Settings::from_env() },
        activity.clone(),
    );
    // The KUKA smartPADs at /smartpad/<robot>/ (each controller's KDM in the cell), the owner's as DCV.
    let smartpad = services::smartpad::SmartPadService::new(
        services::smartpad::Settings {
            owner: args.workspace.clone(),
            ..services::smartpad::Settings::from_env(services::hardware::server_cell_config())
        },
        activity.clone(),
    );
    // Only a cell whose template has smartPADs offers them (a cell without a template: as before).
    let infra = state.infra.clone();
    smartpad.set_gate(Arc::new(move || infra.smartpad_refusal()));

    let mut app = Router::new()
        .route(api::HEALTH, get(health))
        .route(api::CONFIG, get(get_config).put(put_config).post(reload_config))
        .route(api::SECRETS, put(put_secrets))
        .route(api::ZENOH_WS, get(ws_proxy::handler))
        .route(api::ZENOH_NATIVE_WS, get(ws_proxy::native_handler))
        .merge(chat_ws::router())
        .merge(services::cell::router())
        .merge(services::dresser::router())
        .merge(services::orient_sweep::router())
        .merge(services::fitter::router())
        .merge(services::scene_edit::router())
        .merge(services::reach::robot_info::router())
        .merge(services::training::router())
        .merge(services::product::router())
        .merge(services::product_welds::router())
        .merge(services::agentic::router())
        .merge(services::codegen::router())
        .merge(services::process_def::router())
        .merge(services::hardware::router())
        .merge(services::infra::router())
        .merge(services::agent_login::router())
        .merge(services::assets::router())
        .merge(services::view_cmd::router())
        .with_state(state.clone());
    // Product & Task's uploads and catalogue writes likewise (merged after the CORS layer below).
    let product_transfers = services::product::transfer_router()
        .with_state(state)
        .layer(axum::middleware::from_fn_with_state(activity.clone(), activity::middleware));
    if let Some(web_root) = args.web_root {
        let index = web_root.join("index.html");
        // `no-cache` = revalidate on every load (an unchanged file is still a cheap 304). Without
        // it browsers cache the viewer heuristically and keep running an old wasm after a deploy.
        let files = tower::ServiceBuilder::new()
            .layer(tower_http::set_header::SetResponseHeaderLayer::overriding(
                axum::http::header::CACHE_CONTROL,
                axum::http::HeaderValue::from_static("no-cache"),
            ))
            .service(ServeDir::new(&web_root).fallback(ServeFile::new(index)));
        app = app.fallback_service(files);
    }
    // CORS for loopback origins only (`loopback_cors`), so a viewer served from elsewhere on this machine
    // (e.g. `trunk serve`) can talk to us, and another site's page in the same browser can't. Not
    // NICE DCV (merged after the layer): its session token is a login to the Windows IDE, so only
    // the page's own origin may ask for one or reach the gateway (services/dcv.rs `admit`).
    let dcv = services::dcv::router(dcv).layer(axum::middleware::from_fn_with_state(activity.clone(), activity::middleware));
    // WebRTC likewise: its viewer controls the streamed application.
    let rtc = services::rtc::router(rtc).layer(axum::middleware::from_fn_with_state(activity.clone(), activity::middleware));
    // The workspace files likewise: a save writes to disk.
    // The IDE's terminals likewise: a shell on this host.
    let terminals = services::terminal::router(services::terminal::TerminalService::new(files.clone()))
        .layer(axum::middleware::from_fn_with_state(activity.clone(), activity::middleware));
    // The storage routes likewise: presigned URLs and writes of the user's files.
    let storage_routes = storage::http::router(storage.clone(), files.clone(), storage_assets, authz.clone(), catalogue_sync.clone(), args.edge_data)
        .layer(axum::middleware::from_fn_with_state(activity.clone(), activity::middleware));
    let files = services::files::router(files).layer(axum::middleware::from_fn_with_state(activity.clone(), activity::middleware));
    // The smartPADs likewise: one moves its robot.
    let smartpad = services::smartpad::router(smartpad).layer(axum::middleware::from_fn_with_state(activity.clone(), activity::middleware));
    let app = app
        .layer(axum::middleware::from_fn_with_state(activity, activity::middleware))
        .layer(loopback_cors())
        .merge(dcv)
        .merge(rtc)
        .merge(smartpad)
        .merge(files)
        .merge(storage_routes)
        .merge(product_transfers)
        .merge(terminals)
        .layer(TraceLayer::new_for_http());
    // Around the router, not a `Router::layer`, which would only run after routing.
    let workspace: Option<Arc<str>> = args.workspace.map(Into::into);
    let app = tower::Layer::layer(&axum::middleware::from_fn_with_state(workspace, workspace_prefix::middleware), app);
    // Outermost: with FACTORY_OS_ORIGIN_SECRET, a peer other than loopback needs the edge's x-origin-verify for
    // anything, WebSocket upgrades and the workspace prefix's redirects included (origin_verify.rs).
    let app = tower::Layer::layer(&axum::middleware::from_fn_with_state(origin_secret, origin_verify::middleware), app);
    let app = Router::new().fallback_service(app);

    // Last: viewers that see it (re)appear query their state and scenes, which are served by now.
    let _alive = bus::declare_alive(&alive_bus, &boot_id).await?;

    let shutdown = axum_server::Handle::new();
    tokio::spawn({
        let shutdown = shutdown.clone();
        async move {
            shutdown_signal().await;
            tracing::info!("Shutting down");
            shutdown.graceful_shutdown(Some(std::time::Duration::from_secs(5)));
        }
    });

    // The peer address: the workspace files trust a caller without the edge's alias on loopback only.
    let service = app.into_make_service_with_connect_info::<SocketAddr>();
    match (args.tls_cert, args.tls_key) {
        (Some(cert), Some(key)) => {
            let tls = axum_server::tls_rustls::RustlsConfig::from_pem_file(&cert, &key)
                .await
                .with_context(|| format!("loading TLS cert {} / key {}", cert.display(), key.display()))?;
            tracing::info!("Serving https://{} (API, wss://…{})", args.bind, api::ZENOH_WS);
            axum_server::bind_rustls(args.bind, tls).handle(shutdown).serve(service).await?;
        }
        _ => {
            tracing::info!("Serving http://{} (API, ws://…{})", args.bind, api::ZENOH_WS);
            axum_server::bind(args.bind).handle(shutdown).serve(service).await?;
        }
    }

    // Don't leave the cell-runner processes behind.
    shutdown_cell.stop_and_wait(std::time::Duration::from_secs(8)).await;
    shutdown_agentic.shutdown().await;
    shutdown_chat.shutdown();
    // The index is written at most once a second; what was registered since must survive a restart.
    shutdown_assets.save();
    Ok(())
}

async fn migrate_to_s3(args: Args) -> anyhow::Result<()> {
    let config = Arc::new(ConfigStore::open(args.config.unwrap_or_else(paths::default_config_file)));
    let options = storage::StorageOptions {
        mode: args.storage,
        bucket: args.s3_bucket,
        region: args.s3_region,
        root: args.s3_root,
        endpoint: args.s3_endpoint,
    };
    let storage = storage::Storage::open(&options, args.workspace.as_deref(), {
        let config = config.clone();
        move || storage::local::Layout::of_config(&config)
    })
    .await?;
    anyhow::ensure!(storage.mode() == factoryos_protocol::storage::StorageMode::S3, "migrate-to-s3: not in S3 mode (--storage s3 --s3-bucket …)");
    let summary = storage::migrate::migrate(&storage).await?;
    println!("migrate-to-s3: {summary}");
    if let Some(dir) = std::path::Path::new(&config.response().file).parent()
        && summary.failed == 0
    {
        storage::migrate::write_marker(dir, &summary)?;
    }
    anyhow::ensure!(summary.failed == 0, "migrate-to-s3: {} file(s) failed (see above); run it again", summary.failed);
    Ok(())
}

/// `migrate-cells` in S3 mode (`FACTORY_OS_STORAGE=s3`, …: the backend's environment): what it wrote
/// goes to the store, what it removed goes from it. Locally: nothing to do.
async fn migrated_cells_to_store(argv0: &str, outcome: &services::cell_migrate::Outcome) -> anyhow::Result<()> {
    let args = Args::try_parse_from([argv0])?;
    let config = Arc::new(ConfigStore::open(outcome.config_file.clone()));
    let options = storage::StorageOptions {
        mode: args.storage,
        bucket: args.s3_bucket,
        region: args.s3_region,
        root: args.s3_root,
        endpoint: args.s3_endpoint,
    };
    if options.mode.as_deref().map(str::trim) != Some("s3") {
        return Ok(());
    }
    let storage = storage::Storage::open(&options, args.workspace.as_deref(), move || storage::local::Layout::of_config(&config)).await?;
    outcome.touched.write_through(&storage).await;
    println!(
        "The store follows: {} written, {} removed (users/{}/…)",
        outcome.touched.saved.len(),
        outcome.touched.removed.len(),
        args.workspace.as_deref().unwrap_or("?")
    );
    Ok(())
}

async fn shutdown_signal() {
    #[cfg(unix)]
    {
        let mut term = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())
            .expect("failed to install SIGTERM handler");
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {}
            _ = term.recv() => {}
        }
    }
    #[cfg(not(unix))]
    tokio::signal::ctrl_c().await.ok();
}

async fn health(State(state): State<AppState>) -> Json<Health> {
    Json(Health {
        idle_secs: Some(state.activity.idle_secs()),
        open_connections: Some(state.activity.open_connections()),
        ..(*state.health).clone()
    })
}

async fn get_config(State(state): State<AppState>) -> Json<ConfigResponse> {
    // S3 mode: one conditional GET, so another backend's change of the user's settings shows.
    match state.config.refresh().await {
        Ok(true) => state.bus.put_json(keys::CONFIG, &state.config.get()),
        Ok(false) => {}
        Err(err) => tracing::warn!("Config: reading your settings from the store: {err:#}"),
    }
    Json(state.config.response())
}

/// CORS for pages served from this machine only, e.g. a viewer on `trunk serve`'s port.
/// Any method, request header and exposed header, as the permissive layer before it allowed, but only for an
/// `Origin` whose host is loopback. Another site's page open in the same browser gets no CORS headers,
/// so it can neither read a response nor send a preflighted request (a JSON `PUT` or `POST`). No
/// credentials either way.
fn loopback_cors() -> CorsLayer {
    use tower_http::cors::{AllowOrigin, Any};
    CorsLayer::new()
        .allow_origin(AllowOrigin::predicate(|origin, _| loopback_origin(origin.as_bytes())))
        .allow_methods(Any)
        .allow_headers(Any)
        .expose_headers(Any)
}

/// Whether an `Origin` header (`http[s]://host[:port]`) names this machine: `localhost`, 127.0.0.0/8 or `[::1]`.
fn loopback_origin(origin: &[u8]) -> bool {
    let Ok(origin) = std::str::from_utf8(origin) else { return false };
    let Some(rest) = origin.strip_prefix("http://").or_else(|| origin.strip_prefix("https://")) else { return false };
    let (host, port) = match rest.strip_prefix('[') {
        Some(v6) => match v6.split_once(']') {
            Some(split) => split,
            None => return false,
        },
        None => rest.split_at(rest.find(':').unwrap_or(rest.len())),
    };
    let port_ok = port.is_empty()
        || port.strip_prefix(':').is_some_and(|p| !p.is_empty() && p.len() <= 5 && p.bytes().all(|b| b.is_ascii_digit()));
    port_ok && (host.eq_ignore_ascii_case("localhost") || host.parse::<std::net::IpAddr>().is_ok_and(|ip| ip.is_loopback()))
}

/// `PUT /api/v1/config`: S3 mode, the user's settings go to the store first: `409` when another
/// backend changed the same setting meanwhile, `503` when the store didn't take it (nothing saved).
async fn put_config(State(state): State<AppState>, Json(config): Json<Config>) -> Result<Json<ConfigResponse>, (StatusCode, String)> {
    let store = state.config.clone();
    let saved = tokio::task::spawn_blocking(move || store.try_set(config))
        .await
        .map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))?;
    match saved {
        Ok(true) => state.bus.put_json(keys::CONFIG, &state.config.get()),
        Ok(false) => {}
        Err(err @ config_store::SaveError::Conflict(_)) => return Err((StatusCode::CONFLICT, err.to_string())),
        Err(err @ config_store::SaveError::Unavailable(_)) => return Err((StatusCode::SERVICE_UNAVAILABLE, err.to_string())),
    }
    Ok(Json(state.config.response()))
}

/// `POST /api/v1/config`: re-read the file, e.g. after editing it by hand.
async fn reload_config(State(state): State<AppState>) -> Json<ConfigResponse> {
    state.config.reload();
    state.bus.put_json(keys::CONFIG, &state.config.get());
    Json(state.config.response())
}

async fn put_secrets(State(state): State<AppState>, Json(secrets): Json<Secrets>) -> StatusCode {
    state.config.set_secrets(secrets);
    StatusCode::NO_CONTENT
}

#[cfg(test)]
mod loopback_cors_tests {
    #[test]
    fn only_loopback_origins_get_cors() {
        for ok in ["http://localhost:8080", "http://127.0.0.1:3030", "http://127.0.0.2", "https://localhost", "http://[::1]:9902"] {
            assert!(super::loopback_origin(ok.as_bytes()), "{ok}");
        }
        for bad in ["http://localhost.example.com", "http://127.0.0.1.example.com", "https://example.com", "null",
                    "http://example.com@127.0.0.1", "http://[::1", "http://localhost:", "http://10.0.0.1:8080", "http://0.0.0.0:8080"] {
            assert!(!super::loopback_origin(bad.as_bytes()), "{bad}");
        }
    }
}
