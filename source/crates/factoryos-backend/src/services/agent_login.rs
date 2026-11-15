//! Signing the planner agent (`kiro-cli`, see [`super::chat`]) in from the viewer.
//!
//! On a workspace host nobody has a terminal, so the backend runs `kiro-cli login
//! --use-device-flow` itself, as the user it runs as (whose home is the workspace, backed up with
//! it), and hands the device code to the viewer ([`AgentLogin::pending`]). The viewer opens the
//! approval page in a browser window; once the user approves, the login process exits and the
//! agent is signed in.
//!
//! How it signs in is the request's [`LoginMode`] ([`login_command`]): an AWS Builder ID
//! (`--license free`), or an organisation's IAM Identity Center (`--license pro --identity-provider
//! <start URL> --region <region>`; the build's default start URL,
//! [`factoryos_protocol::agent::DEFAULT_START_URL`], when the request names none).
//! `FACTORY_OS_KIRO_LOGIN_ARGS` replaces all of that with its own arguments, whatever the request
//! says ([`AgentLogin::preset`]).

use std::process::Stdio;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use axum::extract::State;
use axum::routing::get;
use axum::{Json, Router};
use factoryos_protocol::agent::{AgentLogin, AgentLoginRequest, DeviceCode, LoginMode};
use factoryos_protocol::api;
use tokio::process::Command;

use crate::AppState;

/// How long the login may take to print its device code.
const CODE_TIMEOUT: Duration = Duration::from_secs(30);
/// The device code's own lifetime is about this; a login still waiting then is given up.
const APPROVAL_TIMEOUT: Duration = Duration::from_secs(15 * 60);
const WHOAMI_TIMEOUT: Duration = Duration::from_secs(15);
/// `kiro-cli login`'s arguments, set by the server: the request's mode, start URL and region are then unused.
const LOGIN_ARGS_ENV: &str = "FACTORY_OS_KIRO_LOGIN_ARGS";

#[derive(Default)]
struct Login {
    pending: Option<DeviceCode>,
    error: Option<String>,
}

#[derive(Default)]
pub struct AgentLoginService {
    login: Arc<Mutex<Login>>,
}

impl AgentLoginService {
    pub fn new() -> Arc<Self> {
        Arc::default()
    }

    async fn state(&self) -> AgentLogin {
        let (pending, error) = {
            let login = self.login.lock().unwrap();
            (login.pending.clone(), login.error.clone())
        };
        let preset = preset_args().is_some();
        if pending.is_some() {
            return AgentLogin { logged_in: false, user: None, pending, error: None, preset };
        }
        let user = whoami().await;
        AgentLogin {
            logged_in: user.is_some(),
            user: user.flatten(),
            pending: None,
            error,
            preset,
        }
    }

    /// Start a device-flow login (unless one is pending or the agent is signed in already) and
    /// wait for its device code.
    async fn start(&self, request: AgentLoginRequest) -> AgentLogin {
        let current = self.state().await;
        if current.logged_in || current.pending.is_some() {
            return current;
        }
        let preset = preset_args();
        let failed = |error: String| AgentLogin { error: Some(error), preset: preset.is_some(), ..AgentLogin::default() };
        let login = match login_command(preset.as_deref(), &request) {
            Ok(login) => login,
            Err(err) => return failed(err),
        };
        match self.spawn(&login).await {
            Ok(code) => AgentLogin { pending: Some(code), preset: preset.is_some(), ..AgentLogin::default() },
            Err(err) => {
                self.login.lock().unwrap().error = Some(err.clone());
                failed(err)
            }
        }
    }

    /// Run `login` ([`login_command`]) and wait for its device code.
    async fn spawn(&self, login: &str) -> Result<DeviceCode, String> {
        // kiro-cli asks for the start URL and region (pre-filled from the flags) even with them,
        // and only on a terminal: off one it reads empty answers and fails ("failed to construct
        // request"). So it runs under `script`'s pseudo-terminal and each prompt is confirmed.
        let mut command = Command::new("script");
        if cfg!(target_os = "macos") {
            command.args(["-q", "/dev/null", "/bin/sh", "-c", login]);
        } else {
            command.args(["-qfec", login, "/dev/null"]);
        }
        let mut child = command
            // `script` runs the command through $SHELL; the service user's is nologin.
            .env("SHELL", "/bin/sh")
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .kill_on_drop(true)
            .spawn()
            .map_err(|err| format!("could not run kiro-cli login: {err}"))?;

        // The pseudo-terminal's output, as lines (split at \r too: prompts and the spinner
        // redraw in place), answering every `? Enter …` prompt with Enter on the way.
        let (lines_tx, mut lines) = tokio::sync::mpsc::unbounded_channel::<String>();
        let mut stdin = child.stdin.take();
        for (stream, answers) in [
            (child.stdout.take().map(|s| Box::new(s) as Box<dyn tokio::io::AsyncRead + Unpin + Send>), true),
            (child.stderr.take().map(|s| Box::new(s) as Box<dyn tokio::io::AsyncRead + Unpin + Send>), false),
        ] {
            let Some(mut stream) = stream else { continue };
            let lines_tx = lines_tx.clone();
            let mut stdin = if answers { stdin.take() } else { None };
            tokio::spawn(async move {
                use tokio::io::{AsyncReadExt as _, AsyncWriteExt as _};
                let (mut seen, mut partial, mut prompts, mut answered) = (String::new(), String::new(), 0, 0);
                let mut buf = [0_u8; 4096];
                while let Ok(n) = stream.read(&mut buf).await {
                    if n == 0 {
                        break;
                    }
                    let text = strip_ansi(&String::from_utf8_lossy(&buf[..n]));
                    seen.push_str(&text);
                    // Keep only a tail to count prompts in (they don't repeat once answered).
                    if seen.len() > 8192 {
                        let cut = seen.len() - 4096;
                        let cut = (cut..seen.len()).find(|i| seen.is_char_boundary(*i)).unwrap_or(0);
                        prompts -= seen[..cut].matches("? Enter").count();
                        answered = answered.min(prompts);
                        seen.drain(..cut);
                    }
                    prompts = prompts.max(seen.matches("? Enter").count());
                    while answered < prompts {
                        if let Some(stdin) = stdin.as_mut() {
                            stdin.write_all(b"\r").await.ok();
                            stdin.flush().await.ok();
                        }
                        answered += 1;
                    }
                    partial.push_str(&text);
                    while let Some(at) = partial.find(['\r', '\n']) {
                        let line: String = partial.drain(..=at).collect();
                        let line = line.trim().to_owned();
                        if !line.is_empty() && !line.contains("Logging in...") {
                            tracing::info!("kiro-cli login: {line}");
                            if lines_tx.send(line).is_err() {
                                return;
                            }
                        }
                    }
                }
            });
        }
        drop(lines_tx);

        let mut output = Vec::new();
        let code = tokio::time::timeout(CODE_TIMEOUT, async {
            let mut code = DeviceCode::default();
            while let Some(line) = lines.recv().await {
                parse_line(&line, &mut code);
                output.push(line);
                if !code.verification_uri.is_empty() && !code.user_code.is_empty() {
                    break;
                }
            }
            code
        })
        .await;
        let code = match code {
            Ok(code) if !code.verification_uri.is_empty() => code,
            Ok(_) | Err(_) => {
                child.kill().await.ok();
                let tail = output.iter().rev().take(5).rev().cloned().collect::<Vec<_>>().join(" ");
                return Err(if tail.is_empty() {
                    "kiro-cli login printed no device code".to_owned()
                } else {
                    format!("kiro-cli login printed no device code: {tail}")
                });
            }
        };

        self.login.lock().unwrap().pending = Some(code.clone());
        self.login.lock().unwrap().error = None;
        let login = self.login.clone();
        tokio::spawn(async move {
            // Keep draining its output so it never blocks on a full pipe.
            let drain = async { while lines.recv().await.is_some() {} };
            let status = tokio::time::timeout(APPROVAL_TIMEOUT, async {
                let (status, ()) = tokio::join!(child.wait(), drain);
                status
            })
            .await;
            let mut login = login.lock().unwrap();
            login.pending = None;
            login.error = match status {
                Ok(Ok(status)) if status.success() => None,
                Ok(Ok(status)) => Some(format!("kiro-cli login failed ({status})")),
                Ok(Err(err)) => Some(format!("kiro-cli login: {err}")),
                Err(_) => Some("the login was not approved in time".to_owned()),
            };
        });
        Ok(code)
    }

    async fn logout(&self) -> AgentLogin {
        let result = Command::new("kiro-cli").arg("logout").stdin(Stdio::null()).output().await;
        if let Err(err) = result {
            self.login.lock().unwrap().error = Some(format!("could not run kiro-cli: {err}"));
        }
        self.state().await
    }
}

/// [`LOGIN_ARGS_ENV`], if set (even empty: `kiro-cli login` with no arguments of ours).
fn preset_args() -> Option<String> {
    std::env::var(LOGIN_ARGS_ENV).ok()
}

/// The shell command line that signs the agent in: `kiro-cli login <args> --use-device-flow`, with
/// `preset` ([`LOGIN_ARGS_ENV`]) as its arguments when set (as written, the request unused), else the
/// request's mode's, once the request is valid: `--license free` for a Builder ID, `--license pro
/// --identity-provider <start URL> --region <region>` (each quoted) for an organisation's.
fn login_command(preset: Option<&str>, request: &AgentLoginRequest) -> Result<String, String> {
    let args = match preset {
        Some(preset) => preset.split_whitespace().collect::<Vec<_>>().join(" "),
        None => {
            request.validate()?;
            match request.mode {
                LoginMode::BuilderId => "--license free".to_owned(),
                LoginMode::IdentityCenter => format!(
                    "--license pro --identity-provider {} --region {}",
                    super::chat::shell_quote(request.start_url.trim()),
                    super::chat::shell_quote(request.region.trim())
                ),
            }
        }
    };
    Ok(format!("kiro-cli login {args} --use-device-flow"))
}

/// `Some(user)` when signed in (`kiro-cli whoami` succeeds), `None` otherwise.
async fn whoami() -> Option<Option<String>> {
    let output = tokio::time::timeout(
        WHOAMI_TIMEOUT,
        Command::new("kiro-cli").args(["whoami", "--format", "json"]).stdin(Stdio::null()).output(),
    )
    .await
    .ok()?
    .ok()?;
    if !output.status.success() {
        return None;
    }
    let text = String::from_utf8_lossy(&output.stdout);
    // The JSON is the first line; the profile follows as plain text.
    let user = text
        .lines()
        .find_map(|line| serde_json::from_str::<serde_json::Value>(line.trim()).ok())
        .and_then(|value| value.get("email").and_then(|email| email.as_str()).map(str::to_owned));
    Some(user)
}

/// Pick the verification URL and the user code out of a line of `kiro-cli login`'s output.
/// Not from its prompts (`? Enter Start URL › https://…`, `✔ …`): that URL is the start page.
fn parse_line(line: &str, code: &mut DeviceCode) {
    if line.starts_with('?') || line.starts_with('✔') || line.contains("Enter Start URL") || line.contains("Enter Region") {
        return;
    }
    for word in line.split_whitespace() {
        let word = word.trim_matches(|c: char| matches!(c, '(' | ')' | '<' | '>' | '"' | '\'' | ',' | '.' | ';'));
        // The device page (with the code) wins over any other URL printed before it.
        let device_page = word.contains("user_code=") || word.contains("/device");
        if word.starts_with("https://") && (code.verification_uri.is_empty() || device_page) {
            code.verification_uri = word.to_owned();
            if let Some(user_code) = word.split_once("user_code=").map(|(_, rest)| rest.split('&').next().unwrap_or(""))
                && is_user_code(user_code)
            {
                code.user_code = user_code.to_owned();
            }
        } else if is_user_code(word) && code.user_code.is_empty() {
            code.user_code = word.to_owned();
        }
    }
}

/// `ABCD-EFGH`: two groups of four letters or digits.
fn is_user_code(word: &str) -> bool {
    let Some((a, b)) = word.split_once('-') else { return false };
    [a, b].iter().all(|part| part.len() == 4 && part.bytes().all(|b| b.is_ascii_uppercase() || b.is_ascii_digit()))
}

fn strip_ansi(line: &str) -> String {
    let mut out = String::with_capacity(line.len());
    let mut chars = line.chars().peekable();
    while let Some(c) = chars.next() {
        if c == '\u{1b}' {
            if chars.peek() == Some(&'[') {
                chars.next();
                for c in chars.by_ref() {
                    if c.is_ascii_alphabetic() {
                        break;
                    }
                }
            }
            continue;
        }
        out.push(c);
    }
    out
}

pub fn router() -> Router<AppState> {
    Router::new().route(api::AGENT_LOGIN, get(get_login).post(post_login).delete(delete_login))
}

async fn get_login(State(state): State<AppState>) -> Json<AgentLogin> {
    Json(state.agent_login.state().await)
}

/// An empty body (older viewers) signs in with the defaults ([`AgentLoginRequest::default`]).
async fn post_login(State(state): State<AppState>, request: Option<Json<AgentLoginRequest>>) -> Json<AgentLogin> {
    let request = request.map(|Json(request)| request).unwrap_or_default();
    Json(state.agent_login.start(request).await)
}

async fn delete_login(State(state): State<AppState>) -> Json<AgentLogin> {
    Json(state.agent_login.logout().await)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The AWS CLI documentation's example start URL.
    const START_URL: &str = "https://my-sso-portal.awsapps.com/start";

    #[test]
    fn parses_device_codes() {
        let mut code = DeviceCode::default();
        for line in [
            "\u{1b}[1mConfirm the following code in the browser\u{1b}[0m",
            "Code: WXYZ-1234",
            "Open this URL: https://device.sso.us-east-1.amazonaws.com/?user_code=WXYZ-1234",
        ] {
            parse_line(&strip_ansi(line), &mut code);
        }
        assert_eq!(code.user_code, "WXYZ-1234");
        assert_eq!(code.verification_uri, "https://device.sso.us-east-1.amazonaws.com/?user_code=WXYZ-1234");

        // kiro-cli 2.26 (an organisation's start URL), as printed on a workspace host, prompts included.
        let mut code = DeviceCode::default();
        for line in [format!("? Enter Start URL › {START_URL}"), format!("✔ Enter Start URL · {START_URL}"),
                     "? Enter Region › us-east-1".into(), "Confirm the following code in the browser".into(), "Code: RZLD-GJXS".into(),
                     format!("Open this URL: {START_URL}/#/device?user_code=RZLD-GJXS")] {
            parse_line(&line, &mut code);
        }
        assert_eq!(code.user_code, "RZLD-GJXS");
        assert_eq!(code.verification_uri, format!("{START_URL}/#/device?user_code=RZLD-GJXS"));

        let mut code = DeviceCode::default();
        parse_line("Visit (https://example.com/device) and enter ABCD-EFGH.", &mut code);
        assert_eq!((code.verification_uri.as_str(), code.user_code.as_str()), ("https://example.com/device", "ABCD-EFGH"));
        assert!(!is_user_code("abcd-efgh") && !is_user_code("ABC-DEFG"));

        assert!(AgentLoginRequest::default().validate().is_ok());
        let bad = |start_url: &str, region: &str| organisation(start_url, region).validate().is_err();
        assert!(bad("http://x.example.com", "us-east-1") && bad("https://a.b/x; rm -rf /", "us-east-1"));
        assert!(bad(START_URL, "us east") && bad(START_URL, ""));
    }

    /// An organisation's sign-in ([`LoginMode::IdentityCenter`]).
    fn organisation(start_url: &str, region: &str) -> AgentLoginRequest {
        AgentLoginRequest { mode: LoginMode::IdentityCenter, start_url: start_url.to_owned(), region: region.to_owned() }
    }

    #[test]
    fn builds_the_login_per_mode() {
        let builder_id = AgentLoginRequest { mode: LoginMode::BuilderId, ..AgentLoginRequest::default() };
        assert_eq!(login_command(None, &builder_id).unwrap(), "kiro-cli login --license free --use-device-flow");
        // A Builder ID ignores whatever start URL and region came along.
        let stray = AgentLoginRequest { mode: LoginMode::BuilderId, start_url: "not a URL".into(), region: "?".into() };
        assert_eq!(login_command(None, &stray).unwrap(), "kiro-cli login --license free --use-device-flow");

        assert_eq!(
            login_command(None, &organisation(&format!(" {START_URL} "), "eu-west-1")).unwrap(),
            format!("kiro-cli login --license pro --identity-provider '{START_URL}' --region 'eu-west-1' --use-device-flow")
        );
        // An organisation's sign-in must name a valid start URL (the public build has no default).
        assert!(login_command(None, &organisation("", "us-east-1")).unwrap_err().contains("no start URL"));
        assert!(login_command(None, &organisation(START_URL, "us east")).is_err());
        // The URL reaches the shell as one word, whatever it holds besides what validate() refuses.
        let odd = format!("{START_URL}/#/x?a=(b)*");
        assert_eq!(
            login_command(None, &organisation(&odd, "us-east-1")).unwrap(),
            format!("kiro-cli login --license pro --identity-provider '{odd}' --region 'us-east-1' --use-device-flow")
        );

        // The build's default sign-in: its start URL, else a Builder ID.
        let default = login_command(None, &AgentLoginRequest::default()).unwrap();
        match factoryos_protocol::agent::DEFAULT_START_URL {
            Some(start_url) => assert!(default.contains(&format!("--license pro --identity-provider '{start_url}' --region 'us-east-1'")), "{default}"),
            None => assert_eq!(default, "kiro-cli login --license free --use-device-flow"),
        }
    }

    #[test]
    fn the_servers_login_args_win() {
        // As set (whitespace collapsed), whatever the request: even one that would not validate.
        let preset = format!("  --license pro   --identity-provider {START_URL} --region eu-west-1 ");
        let expected = format!("kiro-cli login --license pro --identity-provider {START_URL} --region eu-west-1 --use-device-flow");
        assert_eq!(login_command(Some(&preset), &organisation("", "")).unwrap(), expected);
        let builder_id = AgentLoginRequest { mode: LoginMode::BuilderId, ..AgentLoginRequest::default() };
        assert_eq!(login_command(Some(&preset), &builder_id).unwrap(), expected);
        assert_eq!(login_command(Some(""), &organisation("", "")).unwrap(), "kiro-cli login  --use-device-flow");
    }
}
