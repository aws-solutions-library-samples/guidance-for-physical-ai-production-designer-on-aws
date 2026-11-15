//! A client session re-joining a restarted router (as the backend's, with its liveliness token)
//! tells `on_rejoin`, and `follow` queries the latest state again.
#![cfg(not(target_arch = "wasm32"))]

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use factoryos_client::Bus;
use factoryos_client::protocol::keys;

const STATE: &str = "factoryos/test/rejoin/state";

/// A stand-in backend: a router on `port` with the liveliness token of `boot_id`, answering
/// queries on [`STATE`] with `state`.
struct Router {
    session: zenoh::Session,
    _token: zenoh::liveliness::LivelinessToken,
}

impl Router {
    async fn start(port: u16, boot_id: &str, state: &'static str) -> Self {
        let mut config = zenoh::Config::default();
        for (key, value) in [
            ("mode", r#""router""#.to_owned()),
            ("listen/endpoints", format!(r#"["tcp/127.0.0.1:{port}"]"#)),
            ("scouting/multicast/enabled", "false".to_owned()),
        ] {
            config.insert_json5(key, &value).unwrap();
        }
        // The port may take a moment to be free again after the previous router closed.
        let deadline = Instant::now() + Duration::from_secs(10);
        let session = loop {
            match zenoh::open(config.clone()).await {
                Ok(session) => break session,
                Err(err) if Instant::now() < deadline => {
                    eprintln!("router on {port}: {err}; again");
                    tokio::time::sleep(Duration::from_millis(100)).await;
                }
                Err(err) => panic!("router on {port}: {err}"),
            }
        };
        session
            .declare_queryable(STATE)
            .callback(move |query| {
                use zenoh::Wait as _;
                query.reply(STATE, state).wait().ok();
            })
            .background()
            .await
            .unwrap();
        let token = session.liveliness().declare_token(keys::backend_alive(boot_id)).await.unwrap();
        Self { session, _token: token }
    }

    async fn stop(self) {
        self.session.close().await.unwrap();
    }
}

async fn wait_until(what: &str, timeout: Duration, mut done: impl FnMut() -> bool) {
    let deadline = Instant::now() + timeout;
    while !done() {
        assert!(Instant::now() < deadline, "timed out waiting for {what}");
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
}

#[tokio::test(flavor = "multi_thread")]
async fn a_restarted_router_is_a_rejoin() {
    let port = std::net::TcpListener::bind("127.0.0.1:0").unwrap().local_addr().unwrap().port();
    let router = Router::start(port, "boot-a", "a").await;
    let bus = Bus::connect(&format!("tcp/127.0.0.1:{port}")).await.unwrap();
    wait_until("the first token", Duration::from_secs(5), || bus.backend_boot().as_deref() == Some("boot-a")).await;

    let rejoins = Arc::new(AtomicUsize::new(0));
    let count = rejoins.clone();
    bus.on_rejoin(move || {
        count.fetch_add(1, Ordering::SeqCst);
        true
    });
    let seen = Arc::new(Mutex::new(Vec::<String>::new()));
    let sink = seen.clone();
    bus.follow(STATE, move |payload| sink.lock().unwrap().push(String::from_utf8_lossy(&payload).into_owned()))
        .unwrap();
    wait_until("the catch-up query", Duration::from_secs(5), || *seen.lock().unwrap() == ["a"]).await;
    assert_eq!(rejoins.load(Ordering::SeqCst), 0, "joining is not a re-join");

    // Restarted: another boot id. Nothing is published, yet the state is queried again.
    router.stop().await;
    let router = Router::start(port, "boot-b", "b").await;
    wait_until("the re-join", Duration::from_secs(20), || rejoins.load(Ordering::SeqCst) == 1).await;
    assert_eq!(bus.backend_boot().as_deref(), Some("boot-b"));
    wait_until("the state queried again", Duration::from_secs(5), || *seen.lock().unwrap() == ["a", "b"]).await;

    // Back with the same boot id (a reconnect): the token went away and came back.
    router.stop().await;
    let router = Router::start(port, "boot-b", "c").await;
    wait_until("the second re-join", Duration::from_secs(20), || rejoins.load(Ordering::SeqCst) == 2).await;
    wait_until("the state queried again", Duration::from_secs(5), || *seen.lock().unwrap() == ["a", "b", "c"]).await;
    router.stop().await;
}
