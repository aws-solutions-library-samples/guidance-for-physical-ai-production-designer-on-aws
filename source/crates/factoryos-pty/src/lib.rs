//! A shell in a PTY ([`Pty`]): its output and its exit through callbacks, keystrokes written by a
//! thread of its own (a full PTY buffer blocks it, not the caller), its current folder, and killed
//! when dropped. Shared by factoryos-backend's IDE terminals and factoryos-terminal-agent.
//!
//! A shell gets the environment of the process that starts it, but for [`WITHHELD_ENV`].

use std::io::{Read as _, Write as _};
use std::path::{Path, PathBuf};
use std::sync::Mutex;

use portable_pty::{ChildKiller, CommandBuilder, MasterPty, PtySize, native_pty_system};

/// The shell to start and its arguments: `$SHELL -l` (else `/bin/bash -l`); on Windows PowerShell,
/// with a prompt that reports its folder (OSC 7, see [`Osc7`]).
pub fn default_shell() -> (String, Vec<String>) {
    if cfg!(windows) {
        let prompt = "function prompt { $p = $executionContext.SessionState.Path.CurrentLocation.ProviderPath; \
                      \"$([char]27)]7;file://$env:COMPUTERNAME/$($p -replace '\\\\','/')$([char]7)PS $p> \" }";
        return ("powershell.exe".to_owned(), vec!["-NoLogo".to_owned(), "-NoExit".to_owned(), "-Command".to_owned(), prompt.to_owned()]);
    }
    let shell = std::env::var("SHELL").ok().filter(|s| !s.trim().is_empty()).unwrap_or_else(|| "/bin/bash".to_owned());
    (shell, vec!["-l".to_owned()])
}

/// `shell` with the arguments it needs here: a login shell, except PowerShell (its own, above).
pub fn shell_command(shell: &str) -> (String, Vec<String>) {
    let (default, args) = default_shell();
    if shell == default || shell.is_empty() {
        return (default, args);
    }
    let name = shell_name(shell).to_ascii_lowercase();
    let args = if name == "powershell" || name == "pwsh" { default_shell().1 } else if cfg!(windows) { Vec::new() } else { vec!["-l".to_owned()] };
    (shell.to_owned(), args)
}

/// `/bin/zsh` → `zsh`, `C:\…\powershell.exe` → `powershell`.
pub fn shell_name(shell: &str) -> String {
    let base = shell.rsplit(['/', '\\']).next().unwrap_or(shell);
    base.strip_suffix(".exe").unwrap_or(base).to_owned()
}

/// What a shell never inherits: the secrets of the process that starts it, which its user must not read.
/// `KIRO_API_KEY` signs the backend's own planner agent in (`kiro-cli`); `FACTORY_OS_ORIGIN_SECRET` is the
/// value the CloudFront gate sends as `x-origin-verify` (deployment/CONTRACT.md "Origin secret").
pub const WITHHELD_ENV: &[&str] = &["KIRO_API_KEY", "FACTORY_OS_ORIGIN_SECRET"];

pub fn home_dir() -> PathBuf {
    std::env::var_os(if cfg!(windows) { "USERPROFILE" } else { "HOME" }).map_or_else(|| PathBuf::from("/"), PathBuf::from)
}

pub struct Pty {
    master: Mutex<Box<dyn MasterPty + Send>>,
    input: std::sync::mpsc::Sender<Vec<u8>>,
    killer: Mutex<Box<dyn ChildKiller + Send + Sync>>,
    shell_pid: Option<u32>,
    /// The shell's name (`zsh`, `powershell`).
    pub name: String,
}

impl Pty {
    /// `shell args` in `cwd`, `cols` × `rows`: `on_output` gets its output as it comes, `on_exit` its
    /// exit code once it ends (each on a thread of its own).
    pub fn spawn(
        shell: &str,
        args: &[String],
        cwd: &Path,
        (cols, rows): (u16, u16),
        mut on_output: impl FnMut(&[u8]) + Send + 'static,
        on_exit: impl FnOnce(Option<i32>) + Send + 'static,
    ) -> anyhow::Result<Self> {
        let pair = native_pty_system().openpty(size(cols, rows))?;
        let mut child = pair.slave.spawn_command(command(shell, args, cwd))?;
        drop(pair.slave);
        let shell_pid = child.process_id();
        let killer = child.clone_killer();
        let mut reader = pair.master.try_clone_reader()?;
        let mut writer = pair.master.take_writer()?;
        let (input, inputs) = std::sync::mpsc::channel::<Vec<u8>>();
        std::thread::Builder::new().name("pty-in".into()).spawn(move || {
            for bytes in inputs {
                if writer.write_all(&bytes).and_then(|()| writer.flush()).is_err() {
                    break;
                }
            }
        })?;
        std::thread::Builder::new().name("pty-out".into()).spawn(move || {
            let mut buf = [0u8; 8192];
            while let Ok(n) = reader.read(&mut buf) {
                if n == 0 {
                    break;
                }
                on_output(&buf[..n]);
            }
        })?;
        std::thread::Builder::new().name("pty-wait".into()).spawn(move || {
            on_exit(child.wait().ok().map(|status| status.exit_code() as i32));
        })?;
        Ok(Self { master: Mutex::new(pair.master), input, killer: Mutex::new(killer), shell_pid, name: shell_name(shell) })
    }

    pub fn write(&self, bytes: Vec<u8>) {
        let _ = self.input.send(bytes);
    }

    pub fn resize(&self, cols: u16, rows: u16) {
        let _ = self.master.lock().unwrap().resize(size(cols, rows));
    }

    pub fn kill(&self) {
        let _ = self.killer.lock().unwrap().kill();
    }

    /// The folder of the PTY's foreground process (a `cd` in a program run from the shell counts
    /// too), where the system says; `None` elsewhere (then [`Osc7`] may know).
    pub fn cwd(&self) -> Option<String> {
        #[cfg(unix)]
        if let Some(pid) = self.master.lock().unwrap().process_group_leader().filter(|p| *p > 0) {
            return cwd_of(pid as u32);
        }
        self.shell_pid.and_then(cwd_of)
    }
}

impl Drop for Pty {
    fn drop(&mut self) {
        self.kill();
    }
}

/// `shell args` in `cwd`, in this process's environment without [`WITHHELD_ENV`], as a terminal.
fn command(shell: &str, args: &[String], cwd: &Path) -> CommandBuilder {
    let mut cmd = CommandBuilder::new(shell);
    cmd.args(args);
    cmd.cwd(cwd);
    for name in WITHHELD_ENV {
        cmd.env_remove(name);
    }
    cmd.env("TERM", "xterm-256color");
    cmd.env("COLORTERM", "truecolor");
    cmd
}

fn size(cols: u16, rows: u16) -> PtySize {
    PtySize { rows: rows.clamp(2, 500), cols: cols.clamp(2, 1000), pixel_width: 0, pixel_height: 0 }
}

#[cfg(target_os = "linux")]
fn cwd_of(pid: u32) -> Option<String> {
    std::fs::read_link(format!("/proc/{pid}/cwd")).ok().map(|p| p.display().to_string())
}

#[cfg(target_os = "macos")]
fn cwd_of(pid: u32) -> Option<String> {
    // SAFETY: an all-zero proc_vnodepathinfo is valid (plain integers and arrays).
    let mut info: libc::proc_vnodepathinfo = unsafe { std::mem::zeroed() };
    let size = std::mem::size_of::<libc::proc_vnodepathinfo>() as libc::c_int;
    // SAFETY: `info` is `size` bytes, as PROC_PIDVNODEPATHINFO fills.
    let n = unsafe { libc::proc_pidinfo(pid as libc::c_int, libc::PROC_PIDVNODEPATHINFO, 0, (&raw mut info).cast(), size) };
    if n != size {
        return None;
    }
    let bytes: Vec<u8> = info.pvi_cdir.vip_path.iter().flatten().map(|c| *c as u8).take_while(|b| *b != 0).collect();
    String::from_utf8(bytes).ok().filter(|p| !p.is_empty())
}

#[cfg(not(any(target_os = "linux", target_os = "macos")))]
fn cwd_of(_pid: u32) -> Option<String> {
    None
}

/// The folder a shell reports in its output, `ESC ] 7 ; file://host/path` ended by BEL or `ESC \`
/// (split across chunks too).
#[derive(Default)]
pub struct Osc7 {
    pending: Vec<u8>,
}

impl Osc7 {
    const START: &'static [u8] = b"\x1b]7;";
    const MAX: usize = 4096;

    /// The last folder reported in `bytes`, if any.
    pub fn feed(&mut self, bytes: &[u8]) -> Option<String> {
        self.pending.extend_from_slice(bytes);
        let mut found = None;
        loop {
            let Some(start) = find(&self.pending, Self::START) else {
                // Keep what could be the start of one.
                let keep = self.pending.len().min(Self::START.len() - 1);
                self.pending.drain(..self.pending.len() - keep);
                break;
            };
            let body = start + Self::START.len();
            let bel = self.pending[body..].iter().position(|b| *b == 0x07).map(|i| (body + i, 1));
            let st = find(&self.pending[body..], b"\x1b\\").map(|i| (body + i, 2));
            let end = match (bel, st) {
                (Some(b), Some(s)) => Some(if b.0 < s.0 { b } else { s }),
                (b, s) => b.or(s),
            };
            let Some((end, len)) = end else {
                self.pending.drain(..start);
                if self.pending.len() > Self::MAX {
                    self.pending.clear();
                }
                break;
            };
            if let Some(path) = std::str::from_utf8(&self.pending[body..end]).ok().and_then(file_url_path) {
                found = Some(path);
            }
            self.pending.drain(..end + len);
        }
        found
    }
}

fn find(haystack: &[u8], needle: &[u8]) -> Option<usize> {
    haystack.windows(needle.len()).position(|w| w == needle)
}

/// `file://host/C:/Users/x` → `C:/Users/x`, `file://host/home/x%20y` → `/home/x y`.
fn file_url_path(url: &str) -> Option<String> {
    let rest = url.strip_prefix("file://")?;
    let path = &rest[rest.find('/')?..];
    let mut out = Vec::with_capacity(path.len());
    let bytes = path.as_bytes();
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == b'%'
            && let Some(byte) = path.get(i + 1..i + 3).and_then(|h| u8::from_str_radix(h, 16).ok())
        {
            out.push(byte);
            i += 3;
            continue;
        }
        out.push(bytes[i]);
        i += 1;
    }
    let path = String::from_utf8(out).ok()?;
    let drive = path.len() > 2 && path.as_bytes()[2] == b':' && path.as_bytes()[1].is_ascii_alphabetic();
    Some(if drive { path[1..].to_owned() } else { path })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn osc7_paths_across_chunks() {
        let mut osc = Osc7::default();
        assert_eq!(osc.feed(b"PS> \x1b]7;file://WIN-IDE/C:/Users/Admin"), None);
        assert_eq!(osc.feed(b"istrator/Desktop\x07PS C:\\Users> "), Some("C:/Users/Administrator/Desktop".into()));
        assert_eq!(osc.feed(b"x\x1b]7;file://h/home/a%20b\x1b\\ \x1b]7;file://h/tmp\x07"), Some("/tmp".into()), "the last one");
        assert_eq!(osc.feed(b"plain output"), None);
        assert_eq!(osc.feed(b"\x1b]7"), None);
        assert_eq!(osc.feed(b";file://h/srv\x07"), Some("/srv".into()), "a start split after ESC ] 7");
    }

    #[test]
    fn shells_by_name() {
        assert_eq!(shell_name("/bin/zsh"), "zsh");
        assert_eq!(shell_name("C:\\Windows\\System32\\WindowsPowerShell\\v1.0\\powershell.exe"), "powershell");
    }

    #[cfg(unix)]
    #[test]
    fn a_shell_runs_writes_and_ends() {
        let (out_tx, out_rx) = std::sync::mpsc::channel::<Vec<u8>>();
        let (exit_tx, exit_rx) = std::sync::mpsc::channel();
        let dir = std::env::temp_dir();
        let pty = Pty::spawn("/bin/sh", &[], &dir, (80, 24), move |b| { let _ = out_tx.send(b.to_vec()); }, move |c| { let _ = exit_tx.send(c); }).unwrap();
        pty.write(b"echo fos-$((40+2)); exit 7\n".to_vec());
        let mut seen = String::new();
        while !seen.contains("fos-42") {
            seen.push_str(&String::from_utf8_lossy(&out_rx.recv_timeout(std::time::Duration::from_secs(10)).expect("output")));
        }
        assert_eq!(exit_rx.recv_timeout(std::time::Duration::from_secs(10)).unwrap(), Some(7));
    }

    /// Set in the run of this test binary that [`a_shell_never_sees_the_kiro_api_key`] starts.
    const CHILD: &str = "FACTORYOS_PTY_TEST_WITHHELD";

    /// A process with `KIRO_API_KEY` starts a shell: the shell has none. The variable is set for a fresh run
    /// of this test binary (this test only), so no thread here sees the environment change.
    #[cfg(unix)]
    #[test]
    fn a_shell_never_sees_the_kiro_api_key() {
        if std::env::var_os(CHILD).is_none() {
            let name = "tests::a_shell_never_sees_the_kiro_api_key";
            let out = std::process::Command::new(std::env::current_exe().unwrap())
                .args(["--exact", name, "--test-threads=1", "--nocapture"])
                .env(CHILD, "1")
                .env("KIRO_API_KEY", "kiro-key-of-the-backend")
                .output()
                .unwrap();
            let report = format!("{}{}", String::from_utf8_lossy(&out.stdout), String::from_utf8_lossy(&out.stderr));
            assert!(out.status.success() && report.contains("1 passed"), "{report}");
            return;
        }
        // The fresh run: the key is in this process's environment, in no shell's.
        assert_eq!(std::env::var("KIRO_API_KEY").as_deref(), Ok("kiro-key-of-the-backend"));
        let dir = std::env::temp_dir();
        let cmd = command("/bin/sh", &[], &dir);
        assert_eq!(cmd.get_env("KIRO_API_KEY"), None);
        assert_eq!(cmd.get_env("TERM").and_then(|v| v.to_str()), Some("xterm-256color"));
        assert!(cmd.get_env("PATH").is_some(), "the rest is inherited");
        let (out_tx, out_rx) = std::sync::mpsc::channel::<Vec<u8>>();
        let pty = Pty::spawn("/bin/sh", &[], &dir, (80, 24), move |b| { let _ = out_tx.send(b.to_vec()); }, |_| {}).unwrap();
        pty.write(b"echo \"key=[${KIRO_API_KEY-none}]\"; exit 0\n".to_vec());
        let mut seen = String::new();
        // (The terminal echoes the line typed first: `key=[${…}]`.)
        while !seen.contains("key=[none]") && !seen.contains("key=[kiro-key-of-the-backend]") {
            seen.push_str(&String::from_utf8_lossy(&out_rx.recv_timeout(std::time::Duration::from_secs(10)).expect("output")));
        }
        assert!(seen.contains("key=[none]") && !seen.contains("key=[kiro-key-of-the-backend]"), "{seen}");
    }
}
