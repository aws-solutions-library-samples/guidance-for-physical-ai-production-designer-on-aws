//! Runs the real backend binary and drives it the way the viewer does, via factoryos-client.
//!
//! 3D is checked as the viewer sees it: renderer-neutral scenes on `factoryos/scene/<source>` (docs,
//! deltas, joint frames, the late joiner's snapshot) and their files fetched by hash from
//! `GET /api/v1/assets/{sha256}`.
//!
//! Ignored by default: `cell_runs_over_zenoh` and `simulation_runs_beside_a_held_port` need
//! `cell-rerun-server` with its cell-runner embedded, built from this repo's `cell/` workspace by
//! `cell/scripts/build-cell-rerun-server.sh` (to `cell/target/release/cell-rerun-server`, where the
//! backend looks in a checkout; `FACTORY_OS_CELL_RERUN_SERVER` names another, see
//! `paths::cell_server_binary`), and the Dresser / Fitter / asset tests need the starter assets
//! (`assets/catalogue`). Run them all with
//! `cargo test -p factoryos-backend --test end_to_end -- --include-ignored`.

use std::process::{Child, Command};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use factoryos_client::protocol::config::ConfigResponse;
use factoryos_client::protocol::scene::{AssetRef, JointFrame, NodeKind, PoseFrame, SceneDoc, SceneMsg};
use factoryos_client::protocol::{BackendState, BackendStatus, CellCommand, CellResponse, Health, api, keys};
use factoryos_client::{Api, Bus};

struct Backend {
    child: Child,
    api: Api,
    zenoh: String,
    _dir: TempDir,
}

impl Drop for Backend {
    fn drop(&mut self) {
        // SIGTERM, so the backend stops the cell processes it started.
        Command::new("kill").arg(self.child.id().to_string()).status().ok();
        let deadline = Instant::now() + Duration::from_secs(10);
        while Instant::now() < deadline && self.child.try_wait().ok().flatten().is_none() {
            std::thread::sleep(Duration::from_millis(50));
        }
        self.child.kill().ok();
    }
}

struct TempDir(std::path::PathBuf);

impl Drop for TempDir {
    fn drop(&mut self) {
        std::fs::remove_dir_all(&self.0).ok();
    }
}

fn free_port() -> u16 {
    std::net::TcpListener::bind("127.0.0.1:0").unwrap().local_addr().unwrap().port()
}

/// `n` distinct free ports (all bound at once, so they differ from each other).
fn free_ports<const N: usize>() -> [u16; N] {
    let listeners: Vec<_> = (0..N).map(|_| std::net::TcpListener::bind("127.0.0.1:0").unwrap()).collect();
    std::array::from_fn(|i| listeners[i].local_addr().unwrap().port())
}

async fn start_backend() -> Backend {
    let dir = std::env::temp_dir().join(format!("factoryos-e2e-{}-{}", std::process::id(), free_port()));
    std::fs::create_dir_all(&dir).unwrap();
    // Tests run in parallel: a "free" port can be taken by another test's backend before ours
    // binds it, and then ours exits. Try again with new ports.
    for _attempt in 0..5 {
        let [http, zenoh, remote_api, zenoh_ws] = free_ports();
        let mut child = Command::new(env!("CARGO_BIN_EXE_factoryos-backend"))
            .args(["--bind", &format!("127.0.0.1:{http}")])
            .args(["--zenoh-listen", &format!("tcp/127.0.0.1:{zenoh}")])
            .args(["--remote-api-port", &remote_api.to_string()])
            .args(["--zenoh-ws-port", &zenoh_ws.to_string()])
            .args(["--config", dir.join("factoryos.config.json").to_str().unwrap()])
            // Converted meshes and the saved asset index: the test's own.
            .env("FACTORY_OS_STATE_DIR", &dir)
            .spawn()
            .expect("failed to start factoryos-backend");
        let api = Api::new(format!("http://127.0.0.1:{http}"));
        let deadline = Instant::now() + Duration::from_secs(20);
        loop {
            match api.get_async::<Health>(api::HEALTH).await {
                Ok(health) => {
                    let zenoh = factoryos_client::zenoh_endpoint(api.base(), &health).unwrap();
                    return Backend { child, api, zenoh, _dir: TempDir(dir) };
                }
                Err(_) if child.try_wait().ok().flatten().is_some() => break, // exited: a port was taken
                Err(err) if Instant::now() > deadline => panic!("backend did not come up: {err}"),
                Err(_) => tokio::time::sleep(Duration::from_millis(100)).await,
            }
        }
    }
    panic!("backend did not come up: its ports kept being taken");
}

fn query(bus: &Bus, key: &str, payload: Option<Vec<u8>>) -> tokio::sync::oneshot::Receiver<Result<Option<Vec<u8>>, String>> {
    let (tx, rx) = tokio::sync::oneshot::channel();
    bus.query(key, payload, move |reply| {
        tx.send(reply).ok();
    });
    rx
}

async fn http<T: serde::de::DeserializeOwned + Send + 'static>(
    send: impl FnOnce(Box<dyn FnOnce(Result<T, String>) + Send>),
) -> Result<T, String> {
    let (tx, rx) = tokio::sync::oneshot::channel();
    send(Box::new(move |result| {
        tx.send(result).ok();
    }));
    rx.await.unwrap()
}

/// A native viewer that can reach only the HTTP port (a server exposing port 80): its Zenoh session
/// goes over a WebSocket to `/zenoh-native`, through the client's local bridge.
#[tokio::test(flavor = "multi_thread")]
async fn zenoh_over_the_http_port() {
    let backend = start_backend().await;
    let endpoint = format!("{}{}", backend.api.base().replacen("http://", "ws://", 1), api::ZENOH_NATIVE_WS);
    let bus = Bus::connect(&endpoint).await.unwrap();
    let reply = query(&bus, keys::CONFIG, None).await.unwrap().unwrap().expect("no config reply");
    let _: factoryos_client::protocol::Config = serde_json::from_slice(&reply).unwrap();
}

/// The backend's boot id: on `/health`, the answer of the `factoryos/backend/boot` queryable, and
/// in its liveliness token (which re-joins go by, see `Bus::on_rejoin`).
#[tokio::test(flavor = "multi_thread")]
async fn boot_id_on_health_and_zenoh() {
    let backend = start_backend().await;
    let health: Health = backend.api.get_async(api::HEALTH).await.unwrap();
    assert!(!health.boot_id.is_empty());
    let bus = Bus::connect(&backend.zenoh).await.unwrap();
    let reply = query(&bus, keys::BACKEND_BOOT, None).await.unwrap().unwrap().expect("no boot id reply");
    assert_eq!(serde_json::from_slice::<String>(&reply).unwrap(), health.boot_id);
    let deadline = Instant::now() + Duration::from_secs(5);
    while bus.backend_boot().as_deref() != Some(health.boot_id.as_str()) {
        assert!(Instant::now() < deadline, "no liveliness token of {} ({:?})", health.boot_id, bus.backend_boot());
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
}

/// A viewer of workspace `test`, as behind a load balancer: HTTP, Zenoh and the chat all under
/// `/workspace/test/…`.
#[tokio::test(flavor = "multi_thread")]
async fn workspace_prefix() {
    use factoryos_client::chat::{ChatSocket, SocketState};
    let backend = start_backend().await;
    let api = Api::new(factoryos_client::protocol::workspace::url(backend.api.base(), "test"));
    let health: Health = api.get_async(api::HEALTH).await.unwrap();
    let endpoint = factoryos_client::zenoh_endpoint(api.base(), &health).unwrap();
    assert!(endpoint.ends_with("/workspace/test/zenoh-native"), "{endpoint}");
    let bus = Bus::connect(&endpoint).await.unwrap();
    let reply = query(&bus, keys::CONFIG, None).await.unwrap().unwrap().expect("no config reply");
    let _: factoryos_client::protocol::Config = serde_json::from_slice(&reply).unwrap();
    let _: ConfigResponse = api.get_async(api::CONFIG).await.unwrap();

    let mut socket = ChatSocket::connect(api.base(), || {}).unwrap();
    let deadline = Instant::now() + Duration::from_secs(10);
    while socket.state() != SocketState::Open {
        assert!(Instant::now() < deadline, "chat socket did not open under the workspace prefix");
        socket.drain();
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
}

/// The native viewer's planner chat with a backend on another machine: the backend's `/chat`.
#[tokio::test(flavor = "multi_thread")]
async fn native_chat_socket_connects() {
    use factoryos_client::chat::{ChatSocket, SocketState};
    let backend = start_backend().await;
    let mut socket = ChatSocket::connect(backend.api.base(), || {}).unwrap();
    let deadline = Instant::now() + Duration::from_secs(10);
    while socket.state() != SocketState::Open {
        assert!(Instant::now() < deadline, "chat socket did not open");
        socket.drain();
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
}

/// The chat's tabs: sessions created, listed, attached to with `/chat?session=`, and closed (which
/// tells the attached viewer); `/chat` alone stays the default session of older viewers.
#[tokio::test(flavor = "multi_thread")]
async fn chat_sessions_over_http() {
    use factoryos_client::chat::{ChatSocket, SocketState};
    use factoryos_client::protocol::chat::{ChatEvent, ChatSessionInfo, ChatSessions, DEFAULT_SESSION, NewChatSession};
    let backend = start_backend().await;
    let api = backend.api.clone();
    let new = |title: &str| NewChatSession { title: title.to_owned(), prompt: None };
    let a: ChatSessionInfo = http(|done| api.post_json(api::CHAT_SESSIONS, &new("Plan jam2"), done)).await.unwrap();
    let b: ChatSessionInfo = http(|done| api.post_json(api::CHAT_SESSIONS, &new("Diagnose step 3"), done)).await.unwrap();
    assert_ne!(a.id, b.id);
    let list: ChatSessions = api.get_async(api::CHAT_SESSIONS).await.unwrap();
    let titles: Vec<_> = list.sessions.iter().map(|s| s.title.as_str()).collect();
    assert_eq!(titles, ["Plan jam2", "Diagnose step 3"]);

    let mut socket = ChatSocket::connect_session(api.base(), Some(&b.id), || {}).unwrap();
    let deadline = Instant::now() + Duration::from_secs(10);
    let mut events = Vec::new();
    while !events.iter().any(|e| matches!(e, ChatEvent::Transcript { .. })) {
        assert!(Instant::now() < deadline, "no snapshot from session {}", b.id);
        events.extend(socket.drain());
        tokio::time::sleep(Duration::from_millis(50)).await;
    }

    let closed: Result<(), String> = http(|done| api.delete(&api::chat_session(&b.id), done)).await;
    closed.unwrap();
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        if socket.drain().iter().any(|e| matches!(e, ChatEvent::SessionClosed)) {
            break;
        }
        assert!(Instant::now() < deadline, "the attached viewer was not told the session closed");
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
    let gone: Result<(), String> = http(|done| api.delete(&api::chat_session(&b.id), done)).await;
    assert!(gone.unwrap_err().starts_with("404"));

    // An older viewer's `/chat` is the default session, listed with the others.
    let mut legacy = ChatSocket::connect(api.base(), || {}).unwrap();
    let deadline = Instant::now() + Duration::from_secs(10);
    while legacy.state() != SocketState::Open {
        assert!(Instant::now() < deadline, "chat socket did not open");
        legacy.drain();
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        let list: ChatSessions = api.get_async(api::CHAT_SESSIONS).await.unwrap();
        let ids: Vec<_> = list.sessions.iter().map(|s| s.id.clone()).collect();
        if ids == [a.id.clone(), DEFAULT_SESSION.to_owned()] {
            break;
        }
        assert!(Instant::now() < deadline, "sessions: {ids:?}");
        tokio::time::sleep(Duration::from_millis(50)).await;
    }

    // The limit: 6 by default, a clear 409 beyond it.
    for i in 0..4 {
        let _: ChatSessionInfo = http(|done| api.post_json(api::CHAT_SESSIONS, &new(&format!("extra {i}")), done)).await.unwrap();
    }
    let over: Result<ChatSessionInfo, String> = http(|done| api.post_json(api::CHAT_SESSIONS, &new("too many"), done)).await;
    let err = over.unwrap_err();
    assert!(err.starts_with("409") && err.contains("close a tab"), "{err}");
}

/// Product & Task: a product saved, listed and loaded again, and its preview published as a
/// renderer-neutral scene (`factoryos/scene/product`): the part as a box, each task's points, paths
/// and normals.
#[tokio::test(flavor = "multi_thread")]
async fn product_round_trip_and_preview() {
    use factoryos_client::protocol::product::{
        Geometry, LoadProduct, Operation, Params, PreviewProduct, Product, ProductFile, ProductsInfo, Target, Task,
    };
    let backend = start_backend().await;
    let bus = Bus::connect(&backend.zenoh).await.unwrap();
    let product = Product {
        name: "test_cube".to_owned(),
        description: String::new(),
        geometry: Geometry::Box { size: [0.4, 0.3, 0.2] },
        tasks: vec![Task {
            name: "spots".to_owned(),
            process: "spotWeld".to_owned(),
            tool: "spot_gun".to_owned(),
            params: Params::new(),
            operations: vec![
                Operation {
                    id: "W01".to_owned(),
                    target: Target::Point { position: [0.0, 0.0, 0.2], normal: [0.0, 0.0, 1.0] },
                    group: "structural".to_owned(),
                    params: Params::new(),
                },
                Operation {
                    id: "S01".to_owned(),
                    target: Target::Path { points: vec![[-0.1, 0.0, 0.2], [0.1, 0.0, 0.2]], normal: [0.0, 0.0, 1.0] },
                    group: String::new(),
                    params: Params::new(),
                },
            ],
        }],
    };
    let api = backend.api.clone();
    let saved: ProductFile = http(|done| api.post_json(api::PRODUCT_SAVE, &product, done)).await.unwrap();
    assert!(saved.path.ends_with("test_cube/product.json"), "{}", saved.path);

    let listed: ProductsInfo = http(|done| api.get(api::PRODUCT_LIST, done)).await.unwrap();
    assert_eq!(listed.products, ["test_cube"]);
    let request = LoadProduct { name: "test_cube".to_owned() };
    let loaded: ProductFile = http(|done| api.post_json(api::PRODUCT_LOAD, &request, done)).await.unwrap();
    assert_eq!(loaded.product, product);
    let bad = LoadProduct { name: "../etc".to_owned() };
    assert!(http::<ProductFile>(|done| api.post_json(api::PRODUCT_LOAD, &bad, done)).await.is_err());

    let preview = PreviewProduct { product };
    let () = http(|done| api.post_json(api::PRODUCT_PREVIEW, &preview, done)).await.unwrap();
    let doc = wait_for_snapshot(&bus, keys::SCENE_PRODUCT, "the product preview", |doc| doc.node("part").is_some()).await;
    assert_eq!(doc.view.title, "Product & Task");
    let part = doc.node("part").unwrap();
    let NodeKind::Boxes { half_sizes, labels, .. } = &part.kind else { panic!("the part is not a box: {part:?}") };
    assert_close(half_sizes[0], [0.2, 0.15, 0.1]);
    assert_eq!(labels, &["test_cube"]);
    let NodeKind::Points { points, labels, .. } = &doc.node("tasks/0_spots/points").expect("no task points").kind else {
        panic!("tasks/0_spots/points is not points")
    };
    assert_eq!((points.len(), labels.as_slice()), (1, ["spots/W01".to_owned()].as_slice()));
    let NodeKind::Lines { strips, .. } = &doc.node("tasks/0_spots/paths").expect("no task paths").kind else {
        panic!("tasks/0_spots/paths is not lines")
    };
    assert_eq!(strips.len(), 1);
    let NodeKind::Arrows { origins, vectors, .. } = &doc.node("tasks/0_spots/normals").expect("no task normals").kind else {
        panic!("tasks/0_spots/normals is not arrows")
    };
    assert_eq!((origins.len(), vectors.len()), (2, 2));
}

/// The latest doc of scene `source` (its `…/snapshot`), once `ready`.
async fn wait_for_snapshot(bus: &Bus, source: &str, what: &str, ready: impl Fn(&SceneDoc) -> bool) -> SceneDoc {
    let deadline = Instant::now() + Duration::from_secs(30);
    loop {
        let reply = query(bus, &keys::scene_snapshot(source), None).await.unwrap().unwrap();
        if let Some(doc) = reply.map(|bytes| serde_json::from_slice::<SceneDoc>(&bytes).unwrap())
            && ready(&doc)
        {
            return doc;
        }
        assert!(Instant::now() < deadline, "timed out waiting for {what}");
        tokio::time::sleep(Duration::from_millis(200)).await;
    }
}

fn assert_close(actual: [f64; 3], expected: [f64; 3]) {
    assert!(actual.iter().zip(expected).all(|(a, e)| (a - e).abs() < 1e-6), "{actual:?} != {expected:?}");
}

/// `factoryos/assets/register` (fos-live-scene's way to make its files fetchable): a mesh under one
/// of the backend's data roots (here its products folder) is registered and then served by hash;
/// `/etc/passwd`, a relative path, a file outside the roots and a non-asset file are refused.
#[tokio::test(flavor = "multi_thread")]
async fn asset_register_admits_only_assets_under_the_roots() {
    let backend = start_backend().await;
    let bus = Bus::connect(&backend.zenoh).await.unwrap();
    let products = backend._dir.0.join("products/registered");
    std::fs::create_dir_all(&products).unwrap();
    let mesh = products.join("part.obj");
    let obj = b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n";
    std::fs::write(&mesh, obj).unwrap();
    std::fs::write(products.join("product.json"), b"{}").unwrap();
    let outside = backend._dir.0.join("outside.obj");
    std::fs::write(&outside, obj).unwrap();
    let s = |p: &std::path::Path| p.to_string_lossy().into_owned();
    let paths = vec![
        s(&mesh),
        "/etc/passwd".to_owned(),
        "products/registered/part.obj".to_owned(),
        s(&outside),
        s(&products.join("product.json")),
        s(&products.join("../registered/../../outside.obj")),
    ];

    let request = serde_json::to_vec(&paths).unwrap();
    let reply = query(&bus, keys::ASSETS_REGISTER, Some(request)).await.unwrap().unwrap().expect("no reply from factoryos/assets/register");
    let reply: Vec<serde_json::Value> = serde_json::from_slice(&reply).unwrap();
    assert_eq!(reply.len(), paths.len(), "{reply:?}");
    let asset: AssetRef =
        serde_json::from_value(reply[0]["asset"]["asset"].clone()).unwrap_or_else(|err| panic!("{}: {err}: {reply:?}", paths[0]));
    assert_eq!((asset.sha256.as_str(), asset.size, asset.media_type.as_str()), (sha256_hex(obj).as_str(), obj.len() as u64, "model/obj"));
    // A mesh also comes as GLB (one triangle: no LOD), served like it.
    let glb: AssetRef = serde_json::from_value(reply[0]["asset"]["glb"].clone()).unwrap_or_else(|err| panic!("no GLB: {err}: {reply:?}"));
    assert_eq!(glb.media_type, "model/gltf-binary");
    assert!(reply[0]["asset"].get("lod").is_none(), "{reply:?}");
    for (path, answer) in paths.iter().zip(&reply).skip(1) {
        assert!(answer.get("asset").is_none() && answer["error"].is_string(), "{path} was not refused: {answer}");
    }

    // Registered: served by its hash.
    assert_eq!(verify_asset(backend.api.base(), &asset).await, obj);
    assert_eq!(&verify_asset(backend.api.base(), &glb).await[..4], b"glTF");
    // A request that is not a list of paths gets an error, not a registration.
    let bad = query(&bus, keys::ASSETS_REGISTER, Some(br#"{"paths": ["/etc/passwd"]}"#.to_vec())).await.unwrap().unwrap().unwrap();
    let bad: serde_json::Value = serde_json::from_slice(&bad).unwrap();
    assert!(bad["error"].is_string(), "{bad}");
    // Unknown and malformed hashes.
    assert_eq!(get_asset(backend.api.base(), &sha256_hex(b"never registered"), None).await.status(), 404);
    assert_eq!(get_asset(backend.api.base(), "etc-passwd", None).await.status(), 400);
}

#[tokio::test(flavor = "multi_thread")]
async fn config_round_trip_over_http_and_zenoh() {
    let backend = start_backend().await;
    let bus = Bus::connect(&backend.zenoh).await.unwrap();

    let published = Arc::new(Mutex::new(Vec::<String>::new()));
    {
        let published = published.clone();
        bus.subscribe(keys::CONFIG, move |payload| {
            let config: factoryos_client::protocol::Config = serde_json::from_slice(&payload).unwrap();
            published.lock().unwrap().push(config.scene);
        })
        .unwrap();
    }
    tokio::time::sleep(Duration::from_millis(300)).await;

    let api = backend.api.clone();
    let mut response: ConfigResponse = http(|done| api.get(api::CONFIG, done)).await.unwrap();
    assert!(response.file.ends_with("factoryos.config.json"));
    response.config.scene = "e2e.yaml".to_owned();
    let saved: ConfigResponse = http(|done| api.put_json(api::CONFIG, &response.config, done)).await.unwrap();
    assert_eq!(saved.config.scene, "e2e.yaml");
    assert!(std::fs::read_to_string(&saved.file).unwrap().contains("e2e.yaml"));

    // Published to subscribers, and served to late joiners.
    tokio::time::sleep(Duration::from_millis(300)).await;
    assert_eq!(*published.lock().unwrap(), vec!["e2e.yaml".to_owned()]);
    let latest = query(&bus, keys::CONFIG, None).await.unwrap().unwrap().unwrap();
    assert!(String::from_utf8(latest).unwrap().contains("e2e.yaml"));

    // Commands without a running simulation fail cleanly.
    let command = serde_json::to_vec(&CellCommand::new("start")).unwrap();
    let reply = query(&bus, keys::SIM_COMMAND, Some(command)).await.unwrap().unwrap().unwrap();
    let reply: CellResponse = serde_json::from_slice(&reply).unwrap();
    assert!(!reply.ok);
    assert!(reply.error_text().starts_with("the simulation is not running"), "{}", reply.error_text());

    // The live cell's keys are not the backend's: nobody answers them here.
    let live = query(&bus, keys::CELL_STATUS, None).await.unwrap().unwrap();
    assert!(live.is_none(), "the backend answered the live cell's status");
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "needs cell-rerun-server: cell/scripts/build-cell-rerun-server.sh (or FACTORY_OS_CELL_RERUN_SERVER)"]
async fn cell_runs_over_zenoh() {
    let backend = start_backend().await;
    let bus = Bus::connect(&backend.zenoh).await.unwrap();

    let states = Arc::new(Mutex::new(Vec::<serde_json::Value>::new()));
    {
        let states = states.clone();
        bus.subscribe(keys::SIM_STATE, move |payload| {
            states.lock().unwrap().push(serde_json::from_slice(&payload).unwrap());
        })
        .unwrap();
    }
    // The simulation streams no 3D scene of its own (the Robo Fitter's recording shows it), and
    // never on the live cell's keys.
    let live_scene = count_messages(&bus, &[keys::scene(keys::SCENE_CELL), keys::scene_joints(keys::SCENE_CELL)]);
    let live_states = Arc::new(Mutex::new(0_usize));
    {
        let live_states = live_states.clone();
        bus.subscribe(keys::CELL_STATE, move |_| *live_states.lock().unwrap() += 1).unwrap();
    }

    let api = backend.api.clone();
    let status: BackendStatus = http(|done| api.post(api::CELL_START, done)).await.unwrap();
    assert_eq!(status.state, BackendState::Starting);

    let deadline = Instant::now() + Duration::from_secs(90);
    while states.lock().unwrap().is_empty() {
        assert!(Instant::now() < deadline, "no state from the simulation");
        tokio::time::sleep(Duration::from_millis(200)).await;
    }
    let status = query(&bus, keys::SIM_STATUS, None).await.unwrap().unwrap().expect("no simulation status");
    let status: BackendStatus = serde_json::from_slice(&status).unwrap();
    assert_eq!(status.state, BackendState::Ready);
    assert!(status.port.is_some(), "{status:?}");

    let command = |kind: &str, params: Option<serde_json::Value>| {
        let payload = serde_json::to_vec(&CellCommand { kind: kind.to_owned(), params }).unwrap();
        query(&bus, keys::SIM_COMMAND, Some(payload))
    };
    for (kind, params) in [
        ("set_speed", Some(serde_json::json!({ "speed": 2.0 }))),
        ("set_all_ready", None),
        ("start", None),
    ] {
        let reply = command(kind, params).await.unwrap().unwrap().unwrap();
        let reply: CellResponse = serde_json::from_slice(&reply).unwrap();
        assert!(reply.ok, "{kind}: {}", reply.error_text());
    }

    // Running: the state follows.
    tokio::time::sleep(Duration::from_secs(3)).await;
    let last = states.lock().unwrap().last().cloned().unwrap();
    assert_eq!(last["running"], true);
    assert_eq!(last["speed"], 2.0);
    // Late joiners get the latest state.
    let latest = query(&bus, keys::SIM_STATE, None).await.unwrap().unwrap();
    assert!(latest.is_some(), "no late-joiner state");
    assert_eq!(*live_scene.lock().unwrap(), 0, "the backend published on factoryos/scene/cell");
    let snapshot = query(&bus, &keys::scene_snapshot(keys::SCENE_CELL), None).await.unwrap().unwrap();
    assert!(snapshot.is_none(), "the backend answered the live cell's scene snapshot");
    assert_eq!(*live_states.lock().unwrap(), 0, "the simulation wrote factoryos/cell/state");

    let _: BackendStatus = http(|done| api.post(api::CELL_STOP, done)).await.unwrap();
    let deadline = Instant::now() + Duration::from_secs(15);
    loop {
        let status: BackendStatus = http(|done| api.get(api::CELL_STATUS, done)).await.unwrap();
        if status.state == BackendState::Stopped {
            break;
        }
        assert!(Instant::now() < deadline, "cell did not stop: {status:?}");
        tokio::time::sleep(Duration::from_millis(200)).await;
    }
}

/// When the configured cell-runner port is held by another process (on a server connected to the
/// virtual cell, e.g. a cell-runner of its own), the simulation runs on another one instead of
/// failing with "port … is already in use".
#[tokio::test(flavor = "multi_thread")]
#[ignore = "needs cell-rerun-server: cell/scripts/build-cell-rerun-server.sh (or FACTORY_OS_CELL_RERUN_SERVER)"]
async fn simulation_runs_beside_a_held_port() {
    let backend = start_backend().await;
    let api = backend.api.clone();
    let held = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let taken = held.local_addr().unwrap().port();
    let mut response: ConfigResponse = http(|done| api.get(api::CONFIG, done)).await.unwrap();
    response.config.cell_runner_ws_port = taken.into();
    let _: ConfigResponse = http(|done| api.put_json(api::CONFIG, &response.config, done)).await.unwrap();

    let _: BackendStatus = http(|done| api.post(api::CELL_START, done)).await.unwrap();
    let status: BackendStatus = wait_for(&api, api::CELL_STATUS, "the simulation to be ready", |s: &BackendStatus| {
        assert_ne!(s.state, BackendState::Failed, "{:?}", s.error);
        s.state == BackendState::Ready
    })
    .await;
    let port = status.port.expect("no port in the status");
    assert_ne!(port, taken, "the held port");
    drop(held);
    let _: BackendStatus = http(|done| api.post(api::CELL_STOP, done)).await.unwrap();
}

/// Wait until `state` (fetched from `path`) satisfies `done`.
async fn wait_for<T: serde::de::DeserializeOwned + Send + std::fmt::Debug + 'static>(
    api: &Api,
    path: &'static str,
    what: &str,
    done: impl Fn(&T) -> bool,
) -> T {
    let deadline = Instant::now() + Duration::from_secs(60);
    loop {
        let api = api.clone();
        let state: T = http(|cb| api.get(path, cb)).await.unwrap();
        if done(&state) {
            return state;
        }
        assert!(Instant::now() < deadline, "timed out waiting for {what}: {state:?}");
        tokio::time::sleep(Duration::from_millis(200)).await;
    }
}

/// Counts the samples on `keys`.
fn count_messages(bus: &Bus, keys: &[String]) -> Arc<Mutex<usize>> {
    let count = Arc::new(Mutex::new(0));
    for key in keys {
        let count = count.clone();
        bus.subscribe(key, move |_| *count.lock().unwrap() += 1).unwrap();
    }
    count
}

/// Collects the JSON samples on `key`.
fn collect<T: serde::de::DeserializeOwned + Send + 'static>(bus: &Bus, key: &str) -> Arc<Mutex<Vec<T>>> {
    let samples = Arc::new(Mutex::new(Vec::new()));
    let sink = samples.clone();
    bus.subscribe(key, move |payload| {
        let sample = serde_json::from_slice(&payload).unwrap_or_else(|err| panic!("not JSON of the expected type: {err}"));
        sink.lock().unwrap().push(sample);
    })
    .unwrap();
    samples
}

/// Waits until `samples` has one that `pick` picks.
async fn wait_for_sample<T, R>(samples: &Mutex<Vec<T>>, what: &str, pick: impl Fn(&T) -> Option<R>) -> R {
    let deadline = Instant::now() + Duration::from_secs(30);
    loop {
        if let Some(found) = samples.lock().unwrap().iter().find_map(&pick) {
            return found;
        }
        assert!(Instant::now() < deadline, "timed out waiting for {what}");
        tokio::time::sleep(Duration::from_millis(100)).await;
    }
}

/// Every asset `doc` references: URDFs, meshes and their sidecars.
fn asset_refs(doc: &SceneDoc) -> Vec<AssetRef> {
    let mut refs = Vec::new();
    for node in &doc.nodes {
        match &node.kind {
            NodeKind::Model { model, .. } => {
                refs.push(model.urdf.clone());
                for mesh in model.meshes.values() {
                    refs.push(mesh.asset.clone());
                    refs.extend(mesh.sidecars.values().cloned());
                    refs.extend(mesh.glb.iter().chain(&mesh.lod).cloned());
                }
            }
            NodeKind::Mesh { mesh, .. } => {
                refs.push(mesh.asset.clone());
                refs.extend(mesh.sidecars.values().cloned());
                refs.extend(mesh.glb.iter().chain(&mesh.lod).cloned());
            }
            _ => {}
        }
    }
    refs.sort();
    refs.dedup();
    refs
}

/// `GET /api/v1/assets/<sha256>`, without content coding (so the body is the file's bytes).
async fn get_asset(base: &str, sha256: &str, if_none_match: Option<&str>) -> reqwest::Response {
    let mut request = reqwest::Client::new().get(format!("{base}{}", api::asset(sha256)));
    if let Some(tag) = if_none_match {
        request = request.header(reqwest::header::IF_NONE_MATCH, tag);
    }
    request.send().await.unwrap()
}

fn sha256_hex(bytes: &[u8]) -> String {
    use sha2::Digest as _;
    sha2::Sha256::digest(bytes).iter().map(|b| format!("{b:02x}")).collect()
}

/// Fetches `asset` and checks it is what the ref says: its hash, size and media type, immutable
/// and private; and that a revalidation with its `ETag` is a bodyless 304.
async fn verify_asset(base: &str, asset: &AssetRef) -> Vec<u8> {
    let response = get_asset(base, &asset.sha256, None).await;
    assert_eq!(response.status(), 200, "{asset:?}");
    let header = |response: &reqwest::Response, name: reqwest::header::HeaderName| {
        response.headers().get(&name).and_then(|v| v.to_str().ok()).unwrap_or_default().to_owned()
    };
    let cache_control = header(&response, reqwest::header::CACHE_CONTROL);
    assert!(cache_control.contains("private") && cache_control.contains("immutable"), "Cache-Control: {cache_control}");
    assert!(header(&response, reqwest::header::CONTENT_TYPE).starts_with(&asset.media_type), "{asset:?}");
    let etag = header(&response, reqwest::header::ETAG);
    assert!(etag.contains(&asset.sha256), "ETag: {etag}");
    let bytes = response.bytes().await.unwrap().to_vec();
    assert_eq!(bytes.len() as u64, asset.size, "{asset:?}");
    assert_eq!(sha256_hex(&bytes), asset.sha256, "the bytes served for {asset:?}");

    let revalidated = get_asset(base, &asset.sha256, Some(&etag)).await;
    assert_eq!(revalidated.status(), 304, "If-None-Match: {etag}");
    assert!(revalidated.bytes().await.unwrap().is_empty());
    bytes
}

/// The link of `urdf` that is no joint's child.
fn root_link(urdf: &[u8]) -> String {
    let robot = urdf_rs::read_from_string(std::str::from_utf8(urdf).unwrap()).unwrap();
    let root = robot.links.iter().find(|link| !robot.joints.iter().any(|j| j.child.link == link.name));
    root.expect("a URDF without a root link").name.clone()
}

/// Robo Dresser: a preview is a new scene on `factoryos/scene/dresser` (the URDF as a model, by
/// content hash; the reach envelope and TCP on its links), a joint slider a [`JointFrame`], and a
/// late joiner gets one doc with the slider folded in. The URDF is served by its hash.
#[tokio::test(flavor = "multi_thread")]
#[ignore = "needs the starter assets (catalogue)"]
async fn dresser_previews_over_zenoh() {
    use factoryos_client::protocol::dresser::{AssetKind, DresserState, JointUpdate, OpenRequest, PreviewRequest};

    let backend = start_backend().await;
    let bus = Bus::connect(&backend.zenoh).await.unwrap();
    let scenes = collect::<SceneMsg>(&bus, &keys::scene(keys::SCENE_DRESSER));
    let frames = collect::<JointFrame>(&bus, &keys::scene_joints(keys::SCENE_DRESSER));
    tokio::time::sleep(Duration::from_millis(300)).await;

    let api = backend.api.clone();
    let _: DresserState = http(|cb| api.post_json(api::DRESSER_OPEN, &OpenRequest { rescan: false }, cb)).await.unwrap();
    let state: DresserState =
        wait_for(&api, api::DRESSER_STATE, "catalogue scan", |s: &DresserState| s.busy.is_none() && !s.robots.is_empty()).await;
    let (kind, name) = match state.dressed.first() {
        Some(dressed) => (AssetKind::Dressed, dressed.name.clone()),
        None => (AssetKind::Robot, state.robots[0].name.clone()),
    };

    let request = PreviewRequest { kind, name };
    let _: DresserState = http(|cb| api.post_json(api::DRESSER_PREVIEW, &request, cb)).await.unwrap();
    let state: DresserState =
        wait_for(&api, api::DRESSER_STATE, "preview", |s: &DresserState| s.busy.is_none() && s.preview.is_some()).await;
    let preview = state.preview.unwrap();
    assert!(!preview.joints.is_empty(), "{} has no joints", preview.label);

    // The preview goes out whole: the robot model, its joints by name.
    let doc = wait_for_sample(&scenes, "the preview's scene", |msg| match msg {
        SceneMsg::Doc(doc) if doc.node("robot").is_some() => Some(doc.clone()),
        _ => None,
    })
    .await;
    assert_eq!(doc.view.title, "Robo Dresser");
    let robot = doc.node("robot").unwrap();
    let NodeKind::Model { model, joints, .. } = &robot.kind else { panic!("the robot is not a model: {robot:?}") };
    assert_eq!(model.urdf.sha256.len(), 64, "{:?}", model.urdf);
    for joint in &preview.joints {
        assert!(joints.contains_key(&joint.name), "joint {} not in the model's joints {joints:?}", joint.name);
    }

    // The URDF is served by its hash, revalidates with a 304; its root link carries the envelope.
    let urdf = verify_asset(backend.api.base(), &model.urdf).await;
    let root = root_link(&urdf);
    match doc.node("reach_envelope") {
        Some(reach) => {
            assert!(preview.has_reach);
            let parent = reach.parent.as_ref().expect("the reach envelope has no parent");
            assert_eq!((parent.node.as_str(), parent.link.as_deref()), ("robot", Some(root.as_str())));
            assert!(matches!(reach.kind, NodeKind::Mesh { .. }), "{reach:?}");
        }
        None => assert!(!preview.has_reach, "has_reach, but no reach_envelope node"),
    }
    // A dressed robot shows its TCP: a frame on one of its links, with axes in it.
    if kind == AssetKind::Dressed {
        let tcp = doc.node("robot/tcp").expect("a dressed robot without its TCP node");
        let parent = tcp.parent.as_ref().expect("the TCP has no parent");
        assert_eq!(parent.node, "robot");
        assert!(parent.link.is_some(), "the TCP is not on a link");
        assert!(matches!(doc.node("robot/tcp/axes").map(|n| &n.kind), Some(NodeKind::Axes { .. })));
    }

    // A joint slider move goes out as a joint frame of the robot.
    let joint = &preview.joints[0];
    let value = joint.lower + (joint.upper - joint.lower) * 0.3;
    let update = JointUpdate { name: joint.name.clone(), value };
    bus.put(keys::DRESSER_JOINT, serde_json::to_vec(&update).unwrap());
    wait_for_sample(&frames, "the slider's joint frame", |frame| {
        let moved = frame.scene_id == doc.scene_id
            && frame.models.iter().any(|m| m.node == "robot" && m.joints.get(&joint.name) == Some(&value));
        moved.then_some(())
    })
    .await;

    // Late joiners get the one doc, with the slider's value in it.
    let snapshot = wait_for_snapshot(&bus, keys::SCENE_DRESSER, "the slider value in the snapshot", |snapshot| {
        matches!(&snapshot.node("robot").map(|n| &n.kind), Some(NodeKind::Model { joints, .. }) if joints.get(&joint.name) == Some(&value))
    })
    .await;
    assert_eq!(snapshot.scene_id, doc.scene_id);
}

/// Every file a scene references (here a Robo Dresser preview's: URDF, meshes, MTLs, reach hull)
/// is served by `GET /api/v1/assets/<sha256>`, byte for byte; unknown and malformed hashes are not.
#[tokio::test(flavor = "multi_thread")]
#[ignore = "needs the starter assets (catalogue)"]
async fn scene_assets_served_by_hash() {
    use factoryos_client::protocol::dresser::{AssetKind, DresserState, OpenRequest, PreviewRequest};

    let backend = start_backend().await;
    let bus = Bus::connect(&backend.zenoh).await.unwrap();
    let api = backend.api.clone();
    let _: DresserState = http(|cb| api.post_json(api::DRESSER_OPEN, &OpenRequest { rescan: false }, cb)).await.unwrap();
    let state: DresserState =
        wait_for(&api, api::DRESSER_STATE, "catalogue scan", |s: &DresserState| s.busy.is_none() && !s.robots.is_empty()).await;
    let request = match state.dressed.first() {
        Some(dressed) => PreviewRequest { kind: AssetKind::Dressed, name: dressed.name.clone() },
        None => PreviewRequest { kind: AssetKind::Robot, name: state.robots[0].name.clone() },
    };
    let _: DresserState = http(|cb| api.post_json(api::DRESSER_PREVIEW, &request, cb)).await.unwrap();
    let doc = wait_for_snapshot(&bus, keys::SCENE_DRESSER, "the preview", |doc| doc.node("robot").is_some()).await;
    let assets = asset_refs(&doc);
    assert!(assets.len() > 1, "the preview references only {assets:?}");
    // The robot's OBJ meshes come converted (GLB, and LODs of the detailed ones).
    assert!(assets.iter().any(|a| a.media_type == "model/gltf-binary"), "no GLB among {assets:?}");
    for asset in &assets {
        verify_asset(api.base(), asset).await;
    }

    let unknown = sha256_hex(b"not an asset of this backend");
    assert_eq!(get_asset(api.base(), &unknown, None).await.status(), 404);
    assert_eq!(get_asset(api.base(), "..%2F..%2Fetc%2Fpasswd", None).await.status(), 400);
    assert_eq!(get_asset(api.base(), &assets[0].sha256.to_uppercase(), None).await.status(), 400);
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "needs the starter assets (asset root with scenes/)"]
async fn fitter_renders_scene_over_zenoh() {
    use factoryos_client::protocol::fitter::{FitterState, FixtureYaml, LoadRequest, RobotYaml, SceneEdit};

    let backend = start_backend().await;
    let bus = Bus::connect(&backend.zenoh).await.unwrap();
    let scenes = collect::<SceneMsg>(&bus, &keys::scene(keys::SCENE_FITTER));
    let poses = collect::<PoseFrame>(&bus, &keys::scene_poses(keys::SCENE_FITTER));
    tokio::time::sleep(Duration::from_millis(300)).await;

    let api = backend.api.clone();
    let state: FitterState = http(|cb| api.post(api::FITTER_OPEN, cb)).await.unwrap();
    assert_eq!(state.scene_seq, 1);
    // A saved scene if there is one, else a new one with the first dressed robot (never saved).
    let state: FitterState = match state.palette.scenes.first().cloned() {
        Some(file_name) => {
            let state: FitterState =
                http(|cb| api.post_json(api::FITTER_LOAD, &LoadRequest { file_name: file_name.clone() }, cb)).await.unwrap();
            assert_eq!(state.scene_seq, 2);
            assert_eq!(state.scene_file_name, file_name);
            // A loaded scene counts as saved.
            assert_eq!(state.saved, Some(SceneEdit { file_name: file_name.clone(), scene: state.scene.clone() }));
            state
        }
        None => {
            // Starter assets may have no dressed robots yet; a fixture renders just the same.
            let mut scene = state.scene.clone();
            if let Some(name) = state.palette.robots.first() {
                scene.robots.push(RobotYaml {
                    robot_id: name.clone(),
                    dressed_urdf: format!("../urdf-dressed-robots/{name}/{name}.urdf"),
                    position: Default::default(),
                    orientation: Default::default(),
                    home_pose_deg: None,
                    pedestal: None,
                    ..Default::default()
                });
            } else {
                let name = state.palette.fixtures.first().expect("no robots or fixtures in the asset root");
                scene.fixtures.push(FixtureYaml {
                    id: name.clone(),
                    urdf: format!("../urdf-fixture-collection/{name}/{name}.urdf"),
                    position: Default::default(),
                    orientation: Default::default(),
                    ..Default::default()
                });
            }
            let edit = SceneEdit { file_name: state.scene_file_name.clone(), scene: scene.clone() };
            bus.put(keys::FITTER_SCENE, serde_json::to_vec(&edit).unwrap());
            wait_for(&api, api::FITTER_STATE, "item added", |s: &FitterState| s.scene == scene).await
        }
    };
    let file_name = state.scene_file_name.clone();
    let robots = state.scene.robots.len();
    let state: FitterState = wait_for(&api, api::FITTER_STATE, "items loaded", |s: &FitterState| {
        s.robot_dofs.len() == robots && s.status.starts_with("Loaded")
    })
    .await;
    assert!(state.robot_dofs.values().all(|&dof| dof > 0));
    // Every robot and fixture is a model node, placed once loaded.
    let keys_of = |scene: &factoryos_client::protocol::fitter::SceneFile| -> Vec<String> {
        let robots = scene.robots.iter().map(|r| format!("robots/{}", r.robot_id));
        robots.chain(scene.fixtures.iter().map(|f| format!("fixtures/{}", f.id))).collect()
    };
    let items = keys_of(&state.scene);
    let doc = wait_for_snapshot(&bus, keys::SCENE_FITTER, "the robots and fixtures", |doc| {
        items.iter().all(|key| matches!(doc.node(key).map(|n| &n.kind), Some(NodeKind::Model { .. })))
    })
    .await;
    assert!(doc.view.title.starts_with("Robo Fitter · "), "{}", doc.view.title);
    for asset in asset_refs(&doc).iter().filter(|a| a.media_type == "application/xml") {
        verify_asset(api.base(), asset).await;
    }

    // An edit (move the first item) re-renders, without replacing the scene.
    let mut scene = state.scene.clone();
    match scene.robots.first_mut() {
        Some(robot) => robot.position.x += 0.5,
        None => scene.fixtures[0].position.x += 0.5,
    }
    let moved = items[0].clone();
    let x = doc.node(&moved).unwrap().pose.xyz[0];
    let edit = SceneEdit { file_name: file_name.clone(), scene: scene.clone() };
    let before = scenes.lock().unwrap().len();
    bus.put(keys::FITTER_SCENE, serde_json::to_vec(&edit).unwrap());
    let seq = state.scene_seq;
    let state: FitterState = wait_for(&api, api::FITTER_STATE, "edit applied", |s: &FitterState| s.scene == scene).await;
    assert_eq!(state.scene_seq, seq);
    assert_ne!(state.saved.map(|saved| saved.scene), Some(scene), "an unsaved edit must not count as saved");
    // The same scene, edited: the moved node only moves, so its new pose goes as a PoseFrame,
    // not in a delta.
    let pose = wait_for_sample(&poses, "the edit's pose frame", |frame| {
        (frame.scene_id == doc.scene_id)
            .then(|| frame.poses.iter().find(|p| p.node == moved && (p.pose.xyz[0] - x - 0.5).abs() < 1e-6).map(|p| p.pose))
            .flatten()
    })
    .await;
    let snapshot = wait_for_snapshot(&bus, keys::SCENE_FITTER, "the edit in the snapshot", |s| s.node(&moved).is_some_and(|n| n.pose == pose)).await;
    assert_eq!(snapshot.scene_id, doc.scene_id);
    tokio::time::sleep(Duration::from_millis(300)).await;
    let resent = scenes.lock().unwrap()[before..].iter().any(|msg| match msg {
        SceneMsg::Delta(delta) => delta.upsert.iter().any(|n| n.id == moved || n.id == format!("{moved}/pedestal")),
        SceneMsg::Doc(_) => true,
    });
    assert!(!resent, "a move must not re-send the node in a delta (or replace the scene)");
}
