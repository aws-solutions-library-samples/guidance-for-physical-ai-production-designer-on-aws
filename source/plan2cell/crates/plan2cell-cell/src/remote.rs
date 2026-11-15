//! The machines of the cell: run a command, copy a file. Over ssh (the PLC host, a KUKA VC host
//! with a key), over `aws ssm send-command` (a KUKA VC host from anywhere with AWS credentials), or
//! over `kubectl exec` (an EKS cell's pods). [`Remote::exec`] runs one program (an argv, no shell)
//! with bytes on its stdin and returns its exit code: the transport of `factoryos/hook@1`
//! ([`crate::hook`]); `docker exec` ([`Docker`]) is the same for `plan2cell cell hooks-check --docker`.

use std::cell::RefCell;
use std::io::Write as _;
use std::path::Path;
use std::process::{Command, Stdio};
use std::rc::Rc;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use crate::{Error, Result};

pub trait Remote {
    /// Run a shell command on the host; its stdout. A non-zero exit is an error.
    fn run(&mut self, cmd: &str) -> Result<String>;
    /// Copy a local file to a path on the host.
    fn copy(&mut self, local: &Path, remote: &str) -> Result<()>;
    /// Send a command without waiting for it to end; [`Remote::wait`] returns what
    /// [`Remote::run`] would have. Several started commands run at the same time (SSM: one
    /// invocation each, ssh: one process each). Default: run it now.
    fn start(&mut self, cmd: &str) -> Result<Pending> {
        Ok(Pending::Done(self.run(cmd)))
    }
    /// The output of a command [`Remote::start`] sent.
    fn wait(&mut self, p: Pending) -> Result<String> {
        match p {
            Pending::Done(r) => r,
            Pending::Ssm { cmd, .. } | Pending::Child { what: cmd, .. } => Err(Error::Remote(
                format!("{cmd}: started on another kind of remote"),
            )),
        }
    }
    /// Run `argv` (a program and its arguments: no shell line) with `stdin` as its standard input
    /// (then closed), within `limits`. A non-zero exit is not an error here: it is [`Exec::code`].
    /// `Err` only when the program could not be run at all. Default: not supported.
    fn exec(&mut self, argv: &[&str], _stdin: &[u8], _limits: &ExecLimits) -> Result<Exec> {
        Err(Error::Remote(format!(
            "{}: running a program with stdin is not supported on this remote",
            argv.join(" ")
        )))
    }
}

/// What [`Remote::exec`] got back.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Exec {
    /// The exit code (`kubectl exec`: the program's); `None` when it was killed (a signal, the
    /// deadline, too much output).
    pub code: Option<i32>,
    pub stdout: Vec<u8>,
    /// At most [`ExecLimits::max_stderr`] bytes (the rest is read and dropped).
    pub stderr: Vec<u8>,
    /// stdout passed [`ExecLimits::max_stdout`]: the program was killed, `stdout` is cut.
    pub overflow: bool,
    /// The deadline passed: the program was killed.
    pub timed_out: bool,
}

impl Exec {
    /// Exit `code` with this stdout (tests, fakes).
    pub fn exit(code: i32, stdout: impl Into<String>) -> Self {
        Self {
            code: Some(code),
            stdout: stdout.into().into_bytes(),
            ..Self::default()
        }
    }

    pub fn stdout_text(&self) -> String {
        String::from_utf8_lossy(&self.stdout).into_owned()
    }

    pub fn stderr_text(&self) -> String {
        String::from_utf8_lossy(&self.stderr).into_owned()
    }
}

/// Bounds of one [`Remote::exec`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExecLimits {
    /// Killed after this long (`None`: no deadline).
    pub timeout: Option<Duration>,
    /// More stdout than this: killed ([`Exec::overflow`]).
    pub max_stdout: usize,
    /// stderr kept (the rest is dropped).
    pub max_stderr: usize,
}

impl Default for ExecLimits {
    fn default() -> Self {
        Self {
            timeout: Some(Duration::from_secs(60)),
            max_stdout: 1 << 20,
            max_stderr: 64 << 10,
        }
    }
}

/// What a reader thread of [`exec_command`] collected so far, and whether it reached the end.
#[derive(Default)]
struct Collected {
    bytes: std::sync::Mutex<Vec<u8>>,
    done: AtomicBool,
}

impl Collected {
    /// The bytes once the reader is done, or after `grace` (a process the program left behind
    /// may hold the pipe open).
    fn take(&self, grace: Duration) -> Vec<u8> {
        let until = Instant::now() + grace;
        while !self.done.load(Ordering::SeqCst) && Instant::now() < until {
            std::thread::sleep(Duration::from_millis(2));
        }
        std::mem::take(&mut *self.bytes.lock().unwrap_or_else(|e| e.into_inner()))
    }
}

/// Read `r` to its end, keeping at most `max` bytes; past `max`, set `over` (when given) and stop
/// reading (the caller kills the program), else keep reading and drop the rest.
fn read_capped(
    mut r: impl std::io::Read + Send + 'static,
    max: usize,
    over: Option<Arc<AtomicBool>>,
) -> Arc<Collected> {
    let c = Arc::new(Collected::default());
    let out = c.clone();
    std::thread::spawn(move || {
        let mut buf = [0u8; 16 << 10];
        loop {
            let n = match r.read(&mut buf) {
                Ok(0) | Err(_) => break,
                Ok(n) => n,
            };
            let mut kept = out.bytes.lock().unwrap_or_else(|e| e.into_inner());
            let room = max.saturating_sub(kept.len());
            kept.extend_from_slice(&buf[..n.min(room)]);
            if n > room
                && let Some(o) = &over
            {
                o.store(true, Ordering::SeqCst);
                break;
            }
        }
        out.done.store(true, Ordering::SeqCst);
    });
    c
}

/// How long the output of an ended program is still read (its pipes may stay open).
const PIPE_GRACE: Duration = Duration::from_secs(2);

/// Run `c` with `stdin` on its standard input within `limits` ([`Remote::exec`] of the local
/// transports: kubectl, ssh, docker).
pub fn exec_command(mut c: Command, stdin: &[u8], limits: &ExecLimits, what: &str) -> Result<Exec> {
    let mut child = c
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .map_err(|e| Error::Remote(format!("{what}: {e}")))?;
    let mut sin = child.stdin.take().expect("piped");
    let input = stdin.to_vec();
    // A program that never reads its stdin must not block us: the write runs on its own.
    let writer = std::thread::spawn(move || {
        let _ = sin.write_all(&input);
    });
    let over = Arc::new(AtomicBool::new(false));
    let out = read_capped(
        child.stdout.take().expect("piped"),
        limits.max_stdout,
        Some(over.clone()),
    );
    let err = read_capped(child.stderr.take().expect("piped"), limits.max_stderr, None);
    let started = Instant::now();
    let (mut killed, mut timed_out) = (false, false);
    let status = loop {
        match child.try_wait() {
            Ok(Some(s)) => break Some(s),
            Ok(None) => {}
            Err(e) => return Err(Error::Remote(format!("{what}: {e}"))),
        }
        if !killed && over.load(Ordering::SeqCst) {
            let _ = child.kill();
            killed = true;
        }
        if !killed && limits.timeout.is_some_and(|t| started.elapsed() > t) {
            let _ = child.kill();
            killed = true;
            timed_out = true;
        }
        std::thread::sleep(Duration::from_millis(5));
    };
    let stdout = out.take(PIPE_GRACE);
    let stderr = err.take(PIPE_GRACE);
    drop(writer);
    let overflow = over.load(Ordering::SeqCst);
    Ok(Exec {
        code: if killed { None } else { status.and_then(|s| s.code()) },
        stdout,
        stderr,
        overflow,
        timed_out,
    })
}

/// A command [`Remote::start`] sent.
#[derive(Debug)]
pub enum Pending {
    /// It already ran (the default [`Remote::start`]).
    Done(Result<String>),
    /// An SSM invocation: polled with `get-command-invocation`.
    Ssm { id: String, cmd: String },
    /// An ssh process.
    Child {
        child: std::process::Child,
        what: String,
    },
}

/// `ssh` / `scp` to a host of `~/.ssh/config` (non-interactive).
pub struct Ssh {
    pub host: String,
    pub options: Vec<String>,
}

impl Ssh {
    pub fn new(host: impl Into<String>) -> Self {
        Self {
            host: host.into(),
            options: ["BatchMode=yes", "ConnectTimeout=20"]
                .iter()
                .flat_map(|o| ["-o".to_string(), o.to_string()])
                .collect(),
        }
    }

    fn output_of(&self, mut c: Command, what: &str) -> Result<String> {
        let o = c
            .output()
            .map_err(|e| Error::Remote(format!("{what}: {e}")))?;
        if o.status.success() {
            Ok(String::from_utf8_lossy(&o.stdout).into_owned())
        } else {
            Err(Error::Remote(format!(
                "{what}: exit {}: {}",
                o.status.code().unwrap_or(-1),
                String::from_utf8_lossy(&o.stderr).trim()
            )))
        }
    }
}

impl Remote for Ssh {
    fn run(&mut self, cmd: &str) -> Result<String> {
        let mut c = Command::new("ssh");
        c.args(&self.options).arg(&self.host).arg("--").arg(cmd);
        self.output_of(c, &format!("ssh {} {cmd}", self.host))
    }

    fn copy(&mut self, local: &Path, remote: &str) -> Result<()> {
        let mut c = Command::new("scp");
        c.arg("-q")
            .args(&self.options)
            .arg(local)
            .arg(format!("{}:{remote}", self.host));
        self.output_of(
            c,
            &format!("scp {} {}:{remote}", local.display(), self.host),
        )
        .map(|_| ())
    }

    fn start(&mut self, cmd: &str) -> Result<Pending> {
        let what = format!("ssh {} {cmd}", self.host);
        let child = Command::new("ssh")
            .args(&self.options)
            .arg(&self.host)
            .arg("--")
            .arg(cmd)
            .stdin(std::process::Stdio::null())
            .stdout(std::process::Stdio::piped())
            .stderr(std::process::Stdio::piped())
            .spawn()
            .map_err(|e| Error::Remote(format!("{what}: {e}")))?;
        Ok(Pending::Child { child, what })
    }

    fn wait(&mut self, p: Pending) -> Result<String> {
        match p {
            Pending::Child { child, what } => {
                let o = child
                    .wait_with_output()
                    .map_err(|e| Error::Remote(format!("{what}: {e}")))?;
                if o.status.success() {
                    Ok(String::from_utf8_lossy(&o.stdout).into_owned())
                } else {
                    Err(Error::Remote(format!(
                        "{what}: exit {}: {}",
                        o.status.code().unwrap_or(-1),
                        String::from_utf8_lossy(&o.stderr).trim()
                    )))
                }
            }
            Pending::Done(r) => r,
            Pending::Ssm { cmd, .. } => Err(Error::Remote(format!(
                "{cmd}: an SSM command waited for over ssh"
            ))),
        }
    }

    /// ssh always hands its command to the remote user's shell: every argument is quoted for it,
    /// so the program gets exactly `argv`.
    fn exec(&mut self, argv: &[&str], stdin: &[u8], limits: &ExecLimits) -> Result<Exec> {
        let line = argv.iter().map(|a| crate::stages::sh(a)).collect::<Vec<_>>().join(" ");
        let mut c = Command::new("ssh");
        c.args(&self.options).arg(&self.host).arg("--").arg(&line);
        exec_command(c, stdin, limits, &format!("ssh {} {line}", self.host))
    }
}

/// `kubectl exec` into one container of a pod (an EKS cell's PLC agent or KUKA VC: no SSM there), as the container's
/// user (root). `target` is `<namespace>/<pod>/<container>`; `kubeconfig` (else `$KUBECONFIG`, else kubectl's
/// default) gives the cluster and the credentials (the workspace's: namespace-scoped `pods/exec`). Non-interactive:
/// no stdin except for [`Remote::copy`], which streams the file and renames it into place.
pub struct Kubectl {
    pub namespace: String,
    pub pod: String,
    pub container: String,
    pub kubeconfig: Option<String>,
}

impl Kubectl {
    /// `<namespace>/<pod>/<container>`: each a DNS-1123 label.
    pub fn parse(target: &str, kubeconfig: Option<String>) -> Result<Self> {
        let parts: Vec<&str> = target.split('/').collect();
        let label = |s: &str| {
            !s.is_empty()
                && s.len() <= 63
                && s.chars().all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '-')
                && !s.starts_with('-')
                && !s.ends_with('-')
        };
        if parts.len() != 3 || !parts.iter().all(|p| label(p)) {
            return Err(Error::Config(format!(
                "kubectl_pod {target:?}: want <namespace>/<pod>/<container>"
            )));
        }
        Ok(Self {
            namespace: parts[0].into(),
            pod: parts[1].into(),
            container: parts[2].into(),
            kubeconfig,
        })
    }

    /// The arguments of `kubectl exec` running `argv` (no shell of ours in between).
    pub fn exec_args(&self, stdin: bool, argv: &[&str]) -> Vec<String> {
        let mut a = Vec::new();
        if let Some(k) = &self.kubeconfig {
            a.extend(["--kubeconfig".to_string(), k.clone()]);
        }
        a.extend(["exec".to_string()]);
        if stdin {
            a.push("-i".into());
        }
        a.extend([
            "-n".to_string(),
            self.namespace.clone(),
            self.pod.clone(),
            "-c".to_string(),
            self.container.clone(),
            "--".to_string(),
        ]);
        a.extend(argv.iter().map(|s| s.to_string()));
        a
    }

    fn what(&self, cmd: &str) -> String {
        format!("kubectl exec {}/{}/{} {cmd}", self.namespace, self.pod, self.container)
    }
}

/// The script [`Kubectl::copy`] runs in the container: stdin to `$1` (via a temporary file next to it).
pub const KUBECTL_COPY: &str = "set -e; t=\"$1.p2c-tmp\"; cat > \"$t\"; mv -f \"$t\" \"$1\"";

impl Remote for Kubectl {
    fn run(&mut self, cmd: &str) -> Result<String> {
        let mut c = Command::new("kubectl");
        c.args(self.exec_args(false, &["bash", "-c", cmd]))
            .stdin(std::process::Stdio::null());
        output(c, &self.what(cmd))
    }

    fn copy(&mut self, local: &Path, remote: &str) -> Result<()> {
        let what = self.what(&format!("< {} > {remote}", local.display()));
        let file = std::fs::File::open(local).map_err(|e| Error::Remote(format!("{what}: {e}")))?;
        let mut c = Command::new("kubectl");
        c.args(self.exec_args(true, &["sh", "-c", KUBECTL_COPY, "sh", remote]))
            .stdin(file);
        output(c, &what).map(|_| ())
    }

    fn start(&mut self, cmd: &str) -> Result<Pending> {
        let what = self.what(cmd);
        let child = Command::new("kubectl")
            .args(self.exec_args(false, &["bash", "-c", cmd]))
            .stdin(std::process::Stdio::null())
            .stdout(std::process::Stdio::piped())
            .stderr(std::process::Stdio::piped())
            .spawn()
            .map_err(|e| Error::Remote(format!("{what}: {e}")))?;
        Ok(Pending::Child { child, what })
    }

    fn wait(&mut self, p: Pending) -> Result<String> {
        match p {
            Pending::Child { child, what } => {
                let o = child
                    .wait_with_output()
                    .map_err(|e| Error::Remote(format!("{what}: {e}")))?;
                finished(o, &what)
            }
            Pending::Done(r) => r,
            Pending::Ssm { cmd, .. } => Err(Error::Remote(format!(
                "{cmd}: an SSM command waited for over kubectl"
            ))),
        }
    }

    /// `kubectl exec -i ... -- <argv>`: the program's own argv, no shell of ours; kubectl exits
    /// with the program's exit code.
    fn exec(&mut self, argv: &[&str], stdin: &[u8], limits: &ExecLimits) -> Result<Exec> {
        let mut c = Command::new("kubectl");
        c.args(self.exec_args(true, argv));
        exec_command(c, stdin, limits, &self.what(&argv.join(" ")))
    }
}

/// `docker exec` into one running container (`plan2cell cell hooks-check --docker`): as
/// [`Kubectl`], with the container's own user.
pub struct Docker {
    pub container: String,
}

impl Docker {
    /// The arguments of `docker exec` running `argv`.
    pub fn exec_args(&self, stdin: bool, argv: &[&str]) -> Vec<String> {
        let mut a = vec!["exec".to_string()];
        if stdin {
            a.push("-i".into());
        }
        a.push(self.container.clone());
        a.extend(argv.iter().map(|s| s.to_string()));
        a
    }

    fn what(&self, cmd: &str) -> String {
        format!("docker exec {} {cmd}", self.container)
    }
}

impl Remote for Docker {
    fn run(&mut self, cmd: &str) -> Result<String> {
        let mut c = Command::new("docker");
        c.args(self.exec_args(false, &["sh", "-c", cmd])).stdin(Stdio::null());
        output(c, &self.what(cmd))
    }

    fn copy(&mut self, local: &Path, remote: &str) -> Result<()> {
        let what = self.what(&format!("< {} > {remote}", local.display()));
        let file = std::fs::File::open(local).map_err(|e| Error::Remote(format!("{what}: {e}")))?;
        let mut c = Command::new("docker");
        c.args(self.exec_args(true, &["sh", "-c", KUBECTL_COPY, "sh", remote]))
            .stdin(file);
        output(c, &what).map(|_| ())
    }

    fn exec(&mut self, argv: &[&str], stdin: &[u8], limits: &ExecLimits) -> Result<Exec> {
        let mut c = Command::new("docker");
        c.args(self.exec_args(true, argv));
        exec_command(c, stdin, limits, &self.what(&argv.join(" ")))
    }
}

fn output(mut c: Command, what: &str) -> Result<String> {
    let o = c
        .output()
        .map_err(|e| Error::Remote(format!("{what}: {e}")))?;
    finished(o, what)
}

fn finished(o: std::process::Output, what: &str) -> Result<String> {
    if o.status.success() {
        Ok(String::from_utf8_lossy(&o.stdout).into_owned())
    } else {
        Err(Error::Remote(format!(
            "{what}: exit {}: {}",
            o.status.code().unwrap_or(-1),
            String::from_utf8_lossy(&o.stderr).trim()
        )))
    }
}

/// `aws ssm send-command` (document AWS-RunShellScript, as root) to one instance, polled with
/// `get-command-invocation`. The script runs under bash; at most 24000 characters of its stdout
/// come back. A copy is sent inline, in base64 chunks of [`Ssm::CHUNK`] bytes (SSM takes 97 KB
/// of parameters per command).
pub struct Ssm {
    pub instance: String,
    pub region: Option<String>,
    pub profile: Option<String>,
    /// Longest the script may run.
    pub timeout: Duration,
    pub poll: Duration,
}

impl Ssm {
    pub const CHUNK: usize = 45_000;

    pub fn new(instance: impl Into<String>) -> Self {
        Self {
            instance: instance.into(),
            region: None,
            profile: None,
            timeout: Duration::from_secs(600),
            poll: Duration::from_secs(2),
        }
    }

    fn aws(&self, args: &[&str]) -> Result<serde_json::Value> {
        let mut c = Command::new("aws");
        if let Some(p) = &self.profile {
            c.args(["--profile", p]);
        }
        if let Some(r) = &self.region {
            c.args(["--region", r]);
        }
        c.args(args).args(["--output", "json"]);
        let what = format!("aws {}", args.first().copied().unwrap_or(""));
        let o = c
            .output()
            .map_err(|e| Error::Remote(format!("{what}: {e}")))?;
        if !o.status.success() {
            return Err(Error::Remote(format!(
                "{what} {}: exit {}: {}",
                args.get(1).copied().unwrap_or(""),
                o.status.code().unwrap_or(-1),
                String::from_utf8_lossy(&o.stderr).trim()
            )));
        }
        serde_json::from_slice(&o.stdout)
            .map_err(|e| Error::Remote(format!("{what}: unreadable answer: {e}")))
    }

    /// Run `commands` (sh lines) on the instance: (exit code, stdout, stderr).
    pub fn send(&self, commands: &[String]) -> Result<(i32, String, String)> {
        let id = self.submit(commands)?;
        self.poll_until_done(&id)
    }

    /// `send-command` of `commands` (sh lines): the command id.
    pub fn submit(&self, commands: &[String]) -> Result<String> {
        let params = serde_json::json!({
            "commands": commands,
            "executionTimeout": [self.timeout.as_secs().max(30).to_string()],
        });
        let file = std::env::temp_dir().join(format!(
            "plan2cell-ssm-{}-{}.json",
            std::process::id(),
            COUNTER.with(|c| {
                let n = c.get() + 1;
                c.set(n);
                n
            })
        ));
        std::fs::write(&file, params.to_string()).map_err(|e| Error::io(&file, e))?;
        let arg = format!("file://{}", file.display());
        let sent = self.aws(&[
            "ssm",
            "send-command",
            "--instance-ids",
            &self.instance,
            "--document-name",
            "AWS-RunShellScript",
            "--timeout-seconds",
            "60",
            "--parameters",
            &arg,
        ]);
        let _ = std::fs::remove_file(&file);
        let sent = sent?;
        sent["Command"]["CommandId"]
            .as_str()
            .map(str::to_string)
            .ok_or_else(|| Error::Remote("ssm send-command: no CommandId".into()))
    }

    /// Poll a command [`Ssm::submit`] sent until it ends: (exit code, stdout, stderr).
    pub fn poll_until_done(&self, id: &str) -> Result<(i32, String, String)> {
        let deadline = Instant::now() + self.timeout + Duration::from_secs(90);
        loop {
            std::thread::sleep(self.poll);
            let r = self.aws(&[
                "ssm",
                "get-command-invocation",
                "--command-id",
                id,
                "--instance-id",
                &self.instance,
            ]);
            let r = match r {
                // The invocation is not registered yet right after send-command.
                Err(Error::Remote(m)) if m.contains("InvocationDoesNotExist") => {
                    if Instant::now() > deadline {
                        return Err(Error::Remote(format!(
                            "ssm {}: command {id} never registered: {m}",
                            self.instance
                        )));
                    }
                    continue;
                }
                r => r?,
            };
            let status = r["Status"].as_str().unwrap_or("");
            if matches!(status, "Pending" | "InProgress" | "Delayed" | "Cancelling") {
                if Instant::now() > deadline {
                    return Err(Error::Remote(format!(
                        "ssm {}: command {id} still {status} after {:?}",
                        self.instance, self.timeout
                    )));
                }
                continue;
            }
            let code = r["ResponseCode"].as_i64().unwrap_or(-1) as i32;
            let out = r["StandardOutputContent"]
                .as_str()
                .unwrap_or("")
                .to_string();
            let err = r["StandardErrorContent"].as_str().unwrap_or("").to_string();
            if !matches!(status, "Success" | "Failed") {
                return Err(Error::Remote(format!(
                    "ssm {}: command {id} {status}: {}",
                    self.instance,
                    err.trim()
                )));
            }
            return Ok((code, out, err));
        }
    }
}

thread_local! {
    static COUNTER: std::cell::Cell<u64> = const { std::cell::Cell::new(0) };
}

/// The commands that write `data` to `remote` (base64 chunks into `<remote>.part`, then a rename).
pub fn copy_commands(data: &[u8], remote: &str, chunk: usize) -> Vec<Vec<String>> {
    let q = crate::stages::sh(remote);
    let part = crate::stages::sh(&format!("{remote}.part"));
    let mut out = Vec::new();
    let chunks: Vec<&[u8]> = if data.is_empty() {
        vec![&[][..]]
    } else {
        data.chunks(chunk).collect()
    };
    let n = chunks.len();
    for (i, c) in chunks.into_iter().enumerate() {
        let mut cmds = Vec::new();
        if i == 0 {
            cmds.push(format!("mkdir -p \"$(dirname {q})\""));
            cmds.push(format!(": > {part}"));
        }
        cmds.push(format!("echo '{}' | base64 -d >> {part}", base64(c)));
        if i + 1 == n {
            cmds.push(format!("mv -f {part} {q}"));
        }
        out.push(cmds);
    }
    out
}

/// Standard base64 with padding.
pub fn base64(data: &[u8]) -> String {
    const T: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut s = String::with_capacity(data.len().div_ceil(3) * 4);
    for c in data.chunks(3) {
        let b = [c[0], *c.get(1).unwrap_or(&0), *c.get(2).unwrap_or(&0)];
        let v = (u32::from(b[0]) << 16) | (u32::from(b[1]) << 8) | u32::from(b[2]);
        for i in 0..4 {
            if i <= c.len() {
                s.push(T[(v >> (18 - 6 * i) & 63) as usize] as char);
            } else {
                s.push('=');
            }
        }
    }
    s
}

/// Standard base64 (padding optional, blanks and line breaks ignored); `None` on any other
/// character.
pub fn base64_decode(text: &str) -> Option<Vec<u8>> {
    let mut out = Vec::with_capacity(text.len() / 4 * 3);
    let (mut acc, mut bits) = (0u32, 0u32);
    for c in text.bytes().filter(|c| !c.is_ascii_whitespace()) {
        let v = match c {
            b'A'..=b'Z' => c - b'A',
            b'a'..=b'z' => c - b'a' + 26,
            b'0'..=b'9' => c - b'0' + 52,
            b'+' => 62,
            b'/' => 63,
            b'=' => break,
            _ => return None,
        };
        acc = (acc << 6) | u32::from(v);
        bits += 6;
        if bits >= 8 {
            bits -= 8;
            out.push((acc >> bits) as u8);
            acc &= (1 << bits) - 1;
        }
    }
    Some(out)
}

impl Remote for Ssm {
    fn run(&mut self, cmd: &str) -> Result<String> {
        let p = self.start(cmd)?;
        self.wait(p)
    }

    /// SSM has no stdin and no exit code of a single program: `factoryos/hook@1` runs over
    /// `kubectl exec` only (docs/tasks/cell-templates.md E7.1).
    fn exec(&mut self, argv: &[&str], _stdin: &[u8], _limits: &ExecLimits) -> Result<Exec> {
        Err(Error::Config(format!(
            "ssm {}: {} needs kubectl exec (a program with stdin is not supported over SSM)",
            self.instance,
            argv.join(" ")
        )))
    }

    fn start(&mut self, cmd: &str) -> Result<Pending> {
        // AWS-RunShellScript runs sh; hand the command to bash unchanged.
        let line = format!("echo {} | base64 -d | bash", base64(cmd.as_bytes()));
        let id = self.submit(&[line])?;
        Ok(Pending::Ssm {
            id,
            cmd: cmd.to_string(),
        })
    }

    fn wait(&mut self, p: Pending) -> Result<String> {
        let (id, cmd) = match p {
            Pending::Ssm { id, cmd } => (id, cmd),
            Pending::Done(r) => return r,
            Pending::Child { what, .. } => {
                return Err(Error::Remote(format!(
                    "{what}: an ssh process waited for over SSM"
                )));
            }
        };
        let (code, out, err) = self.poll_until_done(&id)?;
        if code == 0 {
            Ok(out)
        } else {
            Err(Error::Remote(format!(
                "ssm {} {cmd}: exit {code}: {}",
                self.instance,
                err.trim()
            )))
        }
    }

    fn copy(&mut self, local: &Path, remote: &str) -> Result<()> {
        let data = std::fs::read(local).map_err(|e| Error::io(local, e))?;
        for cmds in copy_commands(&data, remote, Self::CHUNK) {
            let (code, _, err) = self.send(&cmds)?;
            if code != 0 {
                return Err(Error::Remote(format!(
                    "ssm {} copy {} -> {remote}: exit {code}: {}",
                    self.instance,
                    local.display(),
                    err.trim()
                )));
            }
        }
        Ok(())
    }
}

/// A remote shared with the test that inspects its calls afterwards.
impl<R: Remote> Remote for Rc<RefCell<R>> {
    fn run(&mut self, cmd: &str) -> Result<String> {
        self.borrow_mut().run(cmd)
    }

    fn copy(&mut self, local: &Path, remote: &str) -> Result<()> {
        self.borrow_mut().copy(local, remote)
    }

    fn start(&mut self, cmd: &str) -> Result<Pending> {
        self.borrow_mut().start(cmd)
    }

    fn wait(&mut self, p: Pending) -> Result<String> {
        self.borrow_mut().wait(p)
    }

    fn exec(&mut self, argv: &[&str], stdin: &[u8], limits: &ExecLimits) -> Result<Exec> {
        self.borrow_mut().exec(argv, stdin, limits)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Call {
    Run(String),
    Copy(String, String),
    /// [`Remote::exec`]: the argv and the stdin (as text).
    Exec(Vec<String>, String),
}

/// The fake program behind [`MockRemote::exec`]: argv and stdin to what it answers.
pub type ExecFn = Box<dyn FnMut(&[String], &[u8]) -> Result<Exec>>;

/// Records every call; answers `run` with the output of the first rule whose prefix matches, and
/// `exec` with its [`ExecFn`] (none: exit 0, no output).
#[derive(Default)]
pub struct MockRemote {
    pub calls: Vec<Call>,
    pub rules: Vec<(String, Result<String>)>,
    pub exec_fn: Option<ExecFn>,
    /// What each copy sent: the remote path and the local file's bytes (empty if unreadable).
    pub copied: Vec<(String, Vec<u8>)>,
}

impl MockRemote {
    pub fn answer(mut self, prefix: &str, out: Result<String>) -> Self {
        self.rules.push((prefix.into(), out));
        self
    }

    /// Answer [`Remote::exec`] with `f` (a scripted fake hook).
    pub fn on_exec(mut self, f: impl FnMut(&[String], &[u8]) -> Result<Exec> + 'static) -> Self {
        self.exec_fn = Some(Box::new(f));
        self
    }
}

impl Remote for MockRemote {
    fn run(&mut self, cmd: &str) -> Result<String> {
        self.calls.push(Call::Run(cmd.into()));
        match self.rules.iter().find(|(p, _)| cmd.starts_with(p.as_str())) {
            Some((_, Ok(s))) => Ok(s.clone()),
            Some((_, Err(e))) => Err(Error::Remote(e.to_string())),
            None => Ok(String::new()),
        }
    }

    fn copy(&mut self, local: &Path, remote: &str) -> Result<()> {
        self.calls
            .push(Call::Copy(local.display().to_string(), remote.into()));
        self.copied
            .push((remote.into(), std::fs::read(local).unwrap_or_default()));
        Ok(())
    }

    fn exec(&mut self, argv: &[&str], stdin: &[u8], _limits: &ExecLimits) -> Result<Exec> {
        let argv: Vec<String> = argv.iter().map(|s| s.to_string()).collect();
        self.calls
            .push(Call::Exec(argv.clone(), String::from_utf8_lossy(stdin).into_owned()));
        match &mut self.exec_fn {
            Some(f) => f(argv.as_slice(), stdin),
            None => Ok(Exec::exit(0, "")),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn base64_matches_the_standard() {
        for (i, o) in [
            ("", ""),
            ("f", "Zg=="),
            ("fo", "Zm8="),
            ("foo", "Zm9v"),
            ("foob", "Zm9vYg=="),
            ("fooba", "Zm9vYmE="),
            ("foobar", "Zm9vYmFy"),
        ] {
            assert_eq!(base64(i.as_bytes()), o);
        }
        assert_eq!(base64(&[0xff, 0xfe, 0x00]), "//4A");
        for i in ["", "f", "fo", "foo", "foob", "fooba", "foobar"] {
            assert_eq!(base64_decode(&base64(i.as_bytes())).unwrap(), i.as_bytes());
        }
        assert_eq!(base64_decode("Zm9v\nYmFy").unwrap(), b"foobar");
        assert_eq!(base64_decode("//4A").unwrap(), [0xff, 0xfe, 0x00]);
        assert_eq!(base64_decode("Zm9v*"), None);
    }

    #[test]
    fn a_copy_is_split_into_chunks_and_renamed_at_the_end() {
        let c = copy_commands(b"abcdefg", "/var/lib/x y/f.json", 3);
        assert_eq!(c.len(), 3);
        assert_eq!(c[0][0], "mkdir -p \"$(dirname '/var/lib/x y/f.json')\"");
        assert_eq!(c[0][1], ": > '/var/lib/x y/f.json.part'");
        assert_eq!(
            c[0][2],
            "echo 'YWJj' | base64 -d >> '/var/lib/x y/f.json.part'"
        );
        assert_eq!(c[2].len(), 2);
        assert_eq!(
            c[2][1],
            "mv -f '/var/lib/x y/f.json.part' '/var/lib/x y/f.json'"
        );
        assert_eq!(copy_commands(b"", "/f", 3).len(), 1);
    }

    #[test]
    fn kubectl_exec_arguments() {
        let k = Kubectl::parse("cell-jdoe/n1-plc/agent", Some("/k/config".into())).unwrap();
        assert_eq!(
            k.exec_args(false, &["bash", "-c", "docker ps"]),
            ["--kubeconfig", "/k/config", "exec", "-n", "cell-jdoe", "n1-plc", "-c", "agent", "--", "bash", "-c", "docker ps"]
        );
        let k = Kubectl::parse("cell-jdoe/n1-kvc-1/kvc", None).unwrap();
        assert_eq!(k.exec_args(true, &["sh", "-c", KUBECTL_COPY, "sh", "/tmp/x"])[..7], ["exec", "-i", "-n", "cell-jdoe", "n1-kvc-1", "-c", "kvc"]);
        for bad in ["a/b", "a/b/c/d", "a/B/c", "a//c", "-a/b/c", "a/b/c-"] {
            assert!(Kubectl::parse(bad, None).is_err(), "{bad}");
        }
    }

    #[test]
    fn docker_exec_arguments() {
        let d = Docker { container: "p2c-hooks-check-1".into() };
        assert_eq!(
            d.exec_args(true, &["/opt/factoryos/hook", "describe"]),
            ["exec", "-i", "p2c-hooks-check-1", "/opt/factoryos/hook", "describe"]
        );
    }

    #[test]
    fn exec_passes_stdin_and_keeps_the_exit_code() {
        let mut c = Command::new("sh");
        c.args(["-c", "cat; echo; echo err >&2; exit 3"]);
        let e = exec_command(c, b"{\"a\":1}", &ExecLimits::default(), "sh").unwrap();
        assert_eq!(e.code, Some(3));
        assert_eq!(e.stdout_text(), "{\"a\":1}\n");
        assert_eq!(e.stderr_text(), "err\n");
        assert!(!e.overflow && !e.timed_out);
    }

    #[test]
    fn exec_kills_on_too_much_output_and_at_the_deadline() {
        let mut c = Command::new("sh");
        c.args(["-c", "while :; do echo xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx; done"]);
        let limits = ExecLimits { max_stdout: 4096, ..ExecLimits::default() };
        let e = exec_command(c, b"", &limits, "sh").unwrap();
        assert!(e.overflow && e.code.is_none() && e.stdout.len() <= 4096, "{e:?}");
        let mut c = Command::new("sh");
        c.args(["-c", "exec sleep 5"]);
        let limits = ExecLimits { timeout: Some(Duration::from_millis(100)), ..ExecLimits::default() };
        let t = Instant::now();
        let e = exec_command(c, b"", &limits, "sh").unwrap();
        assert!(e.timed_out && e.code.is_none(), "{e:?}");
        assert!(t.elapsed() < Duration::from_secs(4));
    }

    #[test]
    fn ssm_refuses_exec_with_stdin() {
        let e = Ssm::new("i-1").exec(&["/opt/factoryos/hook", "ready"], b"{}", &ExecLimits::default());
        assert_eq!(e.unwrap_err().exit_code(), 3);
    }
}