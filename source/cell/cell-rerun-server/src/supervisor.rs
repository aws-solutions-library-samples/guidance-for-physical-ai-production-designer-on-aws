//! Backend supervisor: extract the embedded backend and manage the cell-runner
//! child process.
//!
//! The cell-runner (and its shared libraries) is embedded as a single zip at
//! build time (see `build.rs`). On first `start` the archive is extracted into a
//! per-version cache directory under the OS temp dir, then the cell-runner is
//! spawned:
//!
//!   cell-runner --server --cell <cellDir> --env <envDir> --port <cellPort>
//!       serves ws://127.0.0.1:<cellPort>/ws + http://127.0.0.1:<cellPort>/env/
//!
//! It listens on loopback, the cell-runner's default. The child inherits our environment, so
//! `CELL_RUNNER_HOST` set for the caller (factoryos-backend) makes it listen elsewhere; the
//! address has to include loopback (`0.0.0.0`), since this supervisor and the backend connect on
//! 127.0.0.1.
//!
//! The child's stdout is discarded and its stderr is inherited onto OUR stderr,
//! so nothing pollutes our stdout — which is the NDJSON control channel back to
//! the caller (factoryos-backend).

use std::io::{Cursor, Read, Write};
use std::net::TcpStream;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};

use anyhow::{anyhow, bail, Context, Result};
use serde::Deserialize;
use serde_json::{json, Value};

/// The embedded backend archive (bin/cell-runner, lib/*).
/// Empty when the crate was built without `CELL_RERUN_BACKEND_ARCHIVE`.
static BACKEND_ZIP: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/backend.zip"));

/// True when a real backend archive was embedded at build time.
pub fn backends_embedded() -> bool {
    !BACKEND_ZIP.is_empty()
}

fn default_cell_port() -> u16 {
    9902
}

/// Parameters accepted by the `start` control method (and the `run` CLI mode).
/// Wire fields are camelCase (e.g. `cellDir`, `cellPort`).
#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct StartParams {
    /// Path to the cell directory (contains cell.yaml, io_names.json, krl/).
    pub cell_dir: String,
    /// Path to the environment directory (model, scenes, meshes) served at /env.
    pub env_dir: String,
    #[serde(default = "default_cell_port")]
    pub cell_port: u16,
    /// Scene YAML, relative to `env_dir` (or absolute). Forwarded to the
    /// cell-runner as `--scene`.
    ///
    /// The cell-runner needs this for the world base transform of every robot:
    /// WITHOUT it every robot's base defaults to the origin, which makes the
    /// collision checker evaluate an overlapping stack of arms and report
    /// collisions that do not exist in the real cell.
    #[serde(default)]
    pub scene: Option<String>,
}

/// Details about the running cell-runner, echoed back to the caller of `start`.
#[derive(Debug, Clone)]
pub struct RunningInfo {
    pub cell_port: u16,
}

impl RunningInfo {
    pub fn to_json(&self) -> Value {
        json!({
            "ready": true,
            "cellPort": self.cell_port,
            "cellWsUrl": format!("ws://127.0.0.1:{}/ws", self.cell_port),
            "cellHttpUrl": format!("http://127.0.0.1:{}", self.cell_port),
        })
    }
}

/// Owns the extracted backend tree and the cell-runner child process, which
/// serves the controller WebSocket and the `/env` mesh/scene files.
#[derive(Default)]
pub struct Supervisor {
    extracted: Option<PathBuf>,
    cell_runner: Option<Child>,
    info: Option<RunningInfo>,
}

impl Supervisor {
    pub fn new() -> Self {
        Self::default()
    }

    /// True once the cell-runner has been spawned and is believed running.
    pub fn is_running(&self) -> bool {
        self.info.is_some()
    }

    /// JSON status snapshot for the `status` control method.
    pub fn status(&self) -> Value {
        match &self.info {
            Some(info) => {
                let mut v = info.to_json();
                v["running"] = json!(true);
                v
            }
            None => json!({ "running": false, "backendsEmbedded": backends_embedded() }),
        }
    }

    /// Start the cell-runner. Idempotent: if already running, returns the
    /// existing running info without spawning a second process.
    pub fn start(&mut self, params: &StartParams) -> Result<RunningInfo> {
        if let Some(info) = &self.info {
            return Ok(info.clone());
        }
        if !backends_embedded() {
            bail!(
                "no backends are embedded in this build. Rebuild the cell-rerun-server \
                 crate with CELL_RERUN_BACKEND_ARCHIVE pointing at a zip of bin/ + lib/."
            );
        }

        let root = self.ensure_extracted().context("extracting embedded backends")?;
        let bin = root.join("bin");
        let lib = root.join("lib");
        let cell_bin = bin.join("cell-runner");
        if !cell_bin.is_file() {
            bail!("embedded backend missing expected binary: {}", cell_bin.display());
        }

        // Refuse to start when something is ALREADY listening on the cell port.
        //
        // Adopting a stranger's socket is worse than failing: every later
        // request — the controller's commands AND the scene/mesh fetches —
        // would silently go to THAT process instead of the cell we are about to
        // start, against whatever cell directory and scene it happens to have
        // been given. In practice this is an orphan from a previous session: the
        // supervisor kills its child on `stop`, but if its parent goes away
        // without stopping (it crashes, or is killed) the cell-runner is left
        // holding the port.
        if port_is_open(params.cell_port) {
            bail!(
                "port {} is already in use by another process — most likely a \
                 cell-runner left over from an earlier session. This supervisor \
                 will not adopt it, because the simulation would then be \
                 talking to that process's cell rather than this one. Stop it (`pkill -f 'cell-runner --server'`) and \
                 retry, or start with a different `cellPort`.",
                params.cell_port
            );
        }

        // Spawn cell-runner (serves the controller WS + /env static files).
        let port = params.cell_port.to_string();
        let mut args: Vec<&str> = vec![
            "--server",
            "--cell",
            &params.cell_dir,
            "--env",
            &params.env_dir,
            "--port",
            &port,
        ];
        // Forward the scene when we can find it, so robot base transforms (and
        // therefore collision checking) are correct. Resolved env root first,
        // then `scenes/`.
        let scene_path = resolve_scene_file(&params.env_dir, params.scene.as_deref());
        let scene_arg;
        match &scene_path {
            Some(path) => {
                scene_arg = path.to_string_lossy().to_string();
                args.push("--scene");
                args.push(&scene_arg);
            }
            None => {
                if let Some(scene) = params.scene.as_deref() {
                    if !scene.trim().is_empty() {
                        eprintln!(
                            "cell-rerun-server: scene {scene:?} not found under {:?}; \
                             starting without --scene (robot bases default to the origin, \
                             so collision checking will be unreliable)",
                            params.env_dir
                        );
                    }
                }
            }
        }
        let mut cell = spawn_backend(&cell_bin, &args, &lib)
            .context("spawning cell-runner")?;

        // Wait for the WebSocket/HTTP port to accept before reporting ready, so
        // the controller can connect immediately.
        //
        // This watches OUR child while it polls: a cell-runner that dies during
        // startup (bad cell directory, missing URDF, port lost to a race) must
        // be reported as a failure. Probing the port alone cannot tell "my child
        // is serving" from "somebody else is", which is how a dead child used to
        // be reported as ready.
        if let Err(e) = wait_for_child_port(&mut cell, params.cell_port, Duration::from_secs(30)) {
            let _ = cell.kill();
            // Reap it, so a failed start leaves no zombie behind.
            let _ = cell.wait();
            return Err(e).context("cell-runner did not become reachable");
        }

        let info = RunningInfo {
            cell_port: params.cell_port,
        };
        self.cell_runner = Some(cell);
        self.info = Some(info.clone());
        Ok(info)
    }

    /// Terminate the cell-runner. Safe to call when nothing is running.
    pub fn stop(&mut self) {
        if let Some(child) = self.cell_runner.take() {
            kill_child(child);
        }
        self.info = None;
    }

    /// Extract the embedded archive into a per-version temp dir (once), and set
    /// executable bits on the extracted binaries.
    fn ensure_extracted(&mut self) -> Result<PathBuf> {
        let dir = extract_backends()?;
        self.extracted = Some(dir.clone());
        Ok(dir)
    }
}

/// Extract the embedded backend archive into a per-version temp dir (idempotent)
/// and return its root.
fn extract_backends() -> Result<PathBuf> {
    if !backends_embedded() {
        bail!(
            "no backends are embedded in this build. Rebuild the cell-rerun-server \
             crate with CELL_RERUN_BACKEND_ARCHIVE pointing at a zip of bin/ + lib/."
        );
    }
    // Key the cache dir by crate version + archive size so a new build extracts
    // fresh without clobbering a concurrent older one.
    let key = format!(
        "cell-rerun-server-{}-{}",
        env!("CARGO_PKG_VERSION"),
        BACKEND_ZIP.len()
    );
    let dir = std::env::temp_dir().join(key);
    let marker = dir.join(".extracted");
    if !marker.exists() {
        // Unpack into a directory of our own, then rename it into place: supervisors started at
        // the same time (parallel tests, two backends) must not exec a binary another one is still
        // writing (exit 126).
        let staging = dir.with_extension(format!("tmp-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&staging);
        extract_zip(BACKEND_ZIP, &staging).context("unpacking embedded backend archive")?;
        std::fs::write(staging.join(".extracted"), b"ok").context("marking the extracted backends")?;
        let mut moved = std::fs::rename(&staging, &dir);
        if moved.is_err() && !marker.exists() {
            // A partial tree left by an interrupted extraction of an older build: replace it.
            let _ = std::fs::remove_dir_all(&dir);
            moved = std::fs::rename(&staging, &dir);
        }
        if let Err(err) = moved {
            let _ = std::fs::remove_dir_all(&staging);
            // Another supervisor got there first: its tree is complete.
            if !marker.exists() {
                return Err(err).with_context(|| format!("moving the extracted backends to {}", dir.display()));
            }
        }
    }
    Ok(dir)
}

impl Drop for Supervisor {
    fn drop(&mut self) {
        self.stop();
    }
}

/// Spawn a backend process with the shared-library search path set to `lib_dir`,
/// discarding its stdout (so it can't corrupt our NDJSON stdout) and inheriting
/// its stderr onto ours for logging.
fn spawn_backend(bin: &Path, args: &[&str], lib_dir: &Path) -> Result<Child> {
    let lib_var = if cfg!(target_os = "macos") {
        "DYLD_LIBRARY_PATH"
    } else {
        "LD_LIBRARY_PATH"
    };
    let mut lib_path = lib_dir.as_os_str().to_os_string();
    if let Some(existing) = std::env::var_os(lib_var) {
        lib_path.push(":");
        lib_path.push(existing);
    }

    Command::new(bin)
        .args(args)
        .env(lib_var, lib_path)
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::inherit())
        .spawn()
        .with_context(|| format!("failed to spawn {}", bin.display()))
}

/// Locate a scene file for `--scene`, trying the env root first then the legacy
/// `scenes/` subfolder. An absolute `scene` is used verbatim. Returns `None` when unset or not found on disk.
fn resolve_scene_file(env_dir: &str, scene: Option<&str>) -> Option<PathBuf> {
    let scene = scene?.trim();
    if scene.is_empty() {
        return None;
    }
    let direct = Path::new(scene);
    if direct.is_absolute() {
        return direct.is_file().then(|| direct.to_path_buf());
    }
    let env = Path::new(env_dir);
    [env.join(scene), env.join("scenes").join(scene)]
        .into_iter()
        .find(|candidate| candidate.is_file())
}

/// True when something is already accepting connections on 127.0.0.1:port.
fn port_is_open(port: u16) -> bool {
    TcpStream::connect_timeout(
        &(std::net::Ipv4Addr::LOCALHOST, port).into(),
        Duration::from_millis(500),
    )
    .is_ok()
}

/// Wait until `child` is accepting on `port`, or fail.
///
/// Fails as soon as the child exits, rather than continuing to poll: a
/// cell-runner that died during startup is a failure even if the port later
/// opens (which would mean an unrelated process took it).
fn wait_for_child_port(child: &mut Child, port: u16, timeout: Duration) -> Result<()> {
    let deadline = Instant::now() + timeout;
    loop {
        match child.try_wait() {
            Ok(Some(status)) => bail!(
                "cell-runner exited before it began listening on port {port} ({status}). \
                 Its error output is in the backend log — a missing or malformed cell \
                 directory is the usual cause."
            ),
            Ok(None) => {}
            Err(e) => bail!("could not check on the cell-runner process: {e}"),
        }
        if port_is_open(port) {
            return Ok(());
        }
        if Instant::now() >= deadline {
            return Err(anyhow!(
                "port {} did not accept connections within {:?}",
                port,
                timeout
            ));
        }
        std::thread::sleep(Duration::from_millis(200));
    }
}

/// Terminate a child and reap it.
fn kill_child(mut child: Child) {
    let _ = child.kill();
    let _ = child.wait();
}

/// Extract a zip archive (in memory) into `dest`, preserving unix executable
/// bits and defaulting `bin/*` entries to 0o755.
fn extract_zip(bytes: &[u8], dest: &Path) -> Result<()> {
    let mut archive = zip::ZipArchive::new(Cursor::new(bytes))?;
    for i in 0..archive.len() {
        let mut entry = archive.by_index(i)?;
        let rel = match entry.enclosed_name() {
            Some(p) => p.to_path_buf(),
            None => continue, // skip unsafe paths (e.g. containing ..)
        };
        let out = dest.join(&rel);
        if entry.is_dir() {
            std::fs::create_dir_all(&out)?;
            continue;
        }
        if let Some(parent) = out.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let mut buf = Vec::with_capacity(entry.size() as usize);
        entry.read_to_end(&mut buf)?;
        let mut f = std::fs::File::create(&out)
            .with_context(|| format!("creating {}", out.display()))?;
        f.write_all(&buf)?;
        set_executable_if_needed(&out, &rel, entry.unix_mode());
    }
    Ok(())
}

#[cfg(unix)]
fn set_executable_if_needed(out: &Path, rel: &Path, mode: Option<u32>) {
    use std::os::unix::fs::PermissionsExt;
    // Preserve the archived mode when present; otherwise make anything under
    // bin/ executable so the backends can be spawned.
    let m = mode.unwrap_or_else(|| {
        if rel.components().any(|c| c.as_os_str() == "bin") {
            0o755
        } else {
            0o644
        }
    });
    let _ = std::fs::set_permissions(out, std::fs::Permissions::from_mode(m));
}

#[cfg(not(unix))]
fn set_executable_if_needed(_out: &Path, _rel: &Path, _mode: Option<u32>) {
    // Windows: executability is determined by extension, nothing to set.
}

#[cfg(all(test, unix))]
mod extract_zip_tests {
    use std::io::Write;
    use std::os::unix::fs::PermissionsExt;

    use zip::write::SimpleFileOptions;

    /// The embedded backend archive's layout: an executable under bin/ (mode kept), a plain file,
    /// and an entry escaping the destination (skipped).
    #[test]
    fn extracts_files_with_their_modes_and_skips_escaping_paths() {
        let mut zip = zip::ZipWriter::new(std::io::Cursor::new(Vec::new()));
        let opts = SimpleFileOptions::default().compression_method(zip::CompressionMethod::Deflated);
        zip.start_file("bin/cell-runner", opts.unix_permissions(0o755)).unwrap();
        zip.write_all(b"#!/bin/sh\n").unwrap();
        zip.start_file("lib/data.txt", opts.unix_permissions(0o644)).unwrap();
        zip.write_all(b"data").unwrap();
        zip.start_file("../outside.txt", opts).unwrap();
        zip.write_all(b"no").unwrap();
        let bytes = zip.finish().unwrap().into_inner();

        let root = std::env::temp_dir().join(format!("cell-rerun-server-extract-{}", std::process::id()));
        let dest = root.join("dest");
        let _ = std::fs::remove_dir_all(&root);
        super::extract_zip(&bytes, &dest).unwrap();

        let bin = dest.join("bin/cell-runner");
        assert_eq!(std::fs::read(&bin).unwrap(), b"#!/bin/sh\n");
        assert_eq!(std::fs::metadata(&bin).unwrap().permissions().mode() & 0o777, 0o755);
        assert_eq!(std::fs::read(dest.join("lib/data.txt")).unwrap(), b"data");
        assert_eq!(std::fs::metadata(dest.join("lib/data.txt")).unwrap().permissions().mode() & 0o777, 0o644);
        assert!(!root.join("outside.txt").exists());
        std::fs::remove_dir_all(&root).unwrap();
    }
}
