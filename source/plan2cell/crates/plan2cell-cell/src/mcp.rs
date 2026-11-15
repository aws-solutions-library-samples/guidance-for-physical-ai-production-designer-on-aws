//! The CODESYS MCP server (Codesys-MCP-Master) as an API: an rmcp client over streamable HTTP
//! that lists the tools and calls one typed tool at a time.

use std::time::{Duration, Instant};

use rmcp::model::{CallToolRequestParams, ReadResourceRequestParams, ResourceContents};
use rmcp::service::RunningService;
use rmcp::transport::StreamableHttpClientTransport;
use rmcp::transport::streamable_http_client::StreamableHttpClientTransportConfig;
use rmcp::{RoleClient, ServiceExt};
use serde_json::{Map, Value};

use crate::config::CodesysMcp;
use crate::{Error, Result};

/// What a tool answered: its text content and the `isError` flag.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ToolOutput {
    pub text: String,
    pub is_error: bool,
}

impl ToolOutput {
    pub fn ok(text: impl Into<String>) -> Self {
        Self {
            text: text.into(),
            is_error: false,
        }
    }

    /// `isError`, or a reply that starts like one of the server's failure texts (some handlers
    /// answer a refusal without the flag).
    pub fn failed(&self) -> bool {
        let t = self.text.trim_start();
        self.is_error
            || [
                "Operation failed",
                "Error:",
                "Refused",
                "Pre-flight FAILED",
                "Failed ",
            ]
            .iter()
            .any(|p| t.starts_with(p))
    }

    /// The text without the server's `DEBUG:` lines, shortened for an error message. The
    /// bridge's `[mcp-bridge]` lines at the end (the CODESYS dialogs answered and open while the
    /// call ran) are always kept.
    pub fn short(&self) -> String {
        let (bridge, kept): (Vec<&str>, Vec<&str>) = self
            .text
            .lines()
            .filter(|l| !l.trim_start().starts_with("DEBUG:"))
            .partition(|l| l.starts_with(BRIDGE_NOTE));
        let joined = kept.join("\n");
        let t = joined.trim();
        let t = if t.len() > 2000 {
            format!("{}...", &t[..t.floor_char_boundary(2000)])
        } else {
            t.to_string()
        };
        if bridge.is_empty() {
            t
        } else {
            format!("{t}\n{}", bridge.join("\n"))
        }
    }
}

/// How mcp-bridge.cjs starts the answer of a call a CODESYS dialog blocks (a modal message box
/// in the IDE that it does not answer by itself), and its note on a failed call that ended
/// while one blocked (an open one in its grace or with a progress bar is noted without it).
pub const DIALOG: &str = "CODESYS shows a dialog:";

/// The prefix of the notes mcp-bridge.cjs appends to a failed call's answer.
const BRIDGE_NOTE: &str = "[mcp-bridge] ";

/// The open CODESYS dialog an answer names ([`DIALOG`] to the end of its line), if any.
pub fn dialog(text: &str) -> Option<String> {
    text.lines()
        .find_map(|l| l.find(DIALOG).map(|i| l[i..].trim().to_string()))
}

/// A tool of the server: its name and the arguments its input schema has.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ToolInfo {
    pub name: String,
    pub args: Vec<String>,
}

impl ToolInfo {
    pub fn has_arg(&self, arg: &str) -> bool {
        self.args.iter().any(|a| a == arg)
    }
}

/// The CODESYS IDE's tools. The stages only talk to this.
pub trait Tools {
    /// The server's tools (`tools/list`).
    fn list(&mut self) -> Result<Vec<ToolInfo>>;
    /// Call one tool.
    fn call(&mut self, tool: &str, args: &Map<String, Value>) -> Result<ToolOutput>;
    /// Read a resource (`resources/read`): its text contents.
    fn read_resource(&mut self, uri: &str) -> Result<ToolOutput>;
}

/// The client. It connects on first use, so a dry run never touches the network.
pub struct McpTools {
    cfg: CodesysMcp,
    rt: tokio::runtime::Runtime,
    client: Option<RunningService<RoleClient, ()>>,
}

const LIST_TIMEOUT: Duration = Duration::from_secs(60);
/// Between two attempts to reach a bridge that does not answer yet.
const CONNECT_RETRY: Duration = Duration::from_secs(5);
/// The largest answer (one SSE event) taken from the server. rmcp's default is 16 MiB, but
/// `export_project_libraries(includeData)` carries up to the overlay's `maxDataBytes` (16 MiB
/// before base64, ~21.5 MiB as JSON): past the limit rmcp drops the transport ("Transport closed").
const MAX_SSE_EVENT: usize = 32 * 1024 * 1024;

/// A connect error that waiting does not cure: the bridge refused the bearer token. The bridge
/// URL (rmcp and reqwest repeat it in the message) is left out, and `401` counts only as a whole
/// number: a port such as 40151 or 34011 is not a refusal.
fn refused(msg: &str, url: &str) -> bool {
    let m = msg.replace(url, "").to_ascii_lowercase();
    m.split(|c: char| !c.is_ascii_digit()).any(|n| n == "401")
        || m.contains("unauthorized")
        || m.contains("token_env")
}

impl McpTools {
    pub fn new(cfg: CodesysMcp) -> Result<Self> {
        let rt = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(2)
            .enable_all()
            .build()
            .map_err(|e| Error::Mcp(e.to_string()))?;
        Ok(Self {
            cfg,
            rt,
            client: None,
        })
    }

    fn connect(&mut self) -> Result<()> {
        if self.client.is_some() {
            return Ok(());
        }
        // An IDE that is still starting (the bridge listens only once its server answered, and that
        // server only once its CODESYS is up): connection refused, or no answer. Wait for it up to
        // connect_wait_s instead of failing the stage at its first call.
        let deadline = Instant::now() + Duration::from_secs(self.cfg.connect_wait_s);
        let mut waited = false;
        loop {
            match self.connect_once() {
                Ok(c) => {
                    if waited {
                        eprintln!("note: the CODESYS MCP bridge {} answers", self.cfg.url);
                    }
                    self.client = Some(c);
                    return Ok(());
                }
                Err(Error::Mcp(m)) if !refused(&m, &self.cfg.url) && Instant::now() < deadline => {
                    if !waited {
                        eprintln!(
                            "note: waiting up to {} s for the CODESYS MCP bridge ({m})",
                            self.cfg.connect_wait_s
                        );
                        waited = true;
                    }
                    std::thread::sleep(
                        CONNECT_RETRY.min(deadline.saturating_duration_since(Instant::now())),
                    );
                }
                Err(e) => return Err(e),
            }
        }
    }

    fn connect_once(&mut self) -> Result<RunningService<RoleClient, ()>> {
        let mut t = StreamableHttpClientTransportConfig::with_uri(self.cfg.url.clone())
            .max_sse_event_size(MAX_SSE_EVENT);
        if let Some(tok) = self.cfg.token()? {
            t = t.auth_header(tok);
        }
        let url = self.cfg.url.clone();
        // mcp-bridge.cjs on the IDE refuses a request without the bearer token (401).
        let hint = if self.cfg.token_env.is_none() {
            " (cell.toml has no codesys_mcp.token_env; the endpoint requires the bearer token)"
        } else {
            ""
        };
        self.rt.block_on(async move {
            let transport = StreamableHttpClientTransport::from_config(t);
            tokio::time::timeout(LIST_TIMEOUT, ().serve(transport))
                .await
                .map_err(|_| Error::Mcp(format!("{url}: no answer within {LIST_TIMEOUT:?}")))?
                .map_err(|e| Error::Mcp(format!("{url}: {e}{hint}")))
        })
    }

    /// The CODESYS dialogs open on the IDE, from the bridge's `GET /dialogs` next to the MCP
    /// endpoint (`title: text [buttons]`, `; `-separated); None when there are none or the bridge
    /// cannot tell (an older bridge: 404).
    fn open_dialogs(&self) -> Option<String> {
        let mut url = reqwest::Url::parse(&self.cfg.url).ok()?;
        url.set_path("/dialogs");
        url.set_query(None);
        let token = self.cfg.token().ok().flatten();
        let body: Value = self.rt.block_on(async move {
            let mut req = reqwest::Client::new()
                .get(url)
                .timeout(Duration::from_secs(45));
            if let Some(t) = token {
                req = req.bearer_auth(t);
            }
            let r = req.send().await.ok()?.error_for_status().ok()?;
            r.json::<Value>().await.ok()
        })?;
        let list = describe_dialogs(&body);
        (!list.is_empty()).then_some(list)
    }

    pub fn close(&mut self) {
        if let Some(c) = self.client.take() {
            let _ = self.rt.block_on(c.cancel());
        }
    }
}

impl Drop for McpTools {
    fn drop(&mut self) {
        self.close();
    }
}

impl Tools for McpTools {
    fn list(&mut self) -> Result<Vec<ToolInfo>> {
        self.connect()?;
        let c = self.client.as_ref().expect("connected");
        let tools = self.rt.block_on(async {
            tokio::time::timeout(LIST_TIMEOUT, c.list_all_tools())
                .await
                .map_err(|_| Error::Mcp("tools/list timed out".into()))?
                .map_err(|e| Error::Mcp(format!("tools/list: {e}")))
        })?;
        Ok(tools
            .into_iter()
            .map(|t| ToolInfo {
                name: t.name.to_string(),
                args: t
                    .input_schema
                    .get("properties")
                    .and_then(Value::as_object)
                    .map(|p| p.keys().cloned().collect())
                    .unwrap_or_default(),
            })
            .collect())
    }

    fn call(&mut self, tool: &str, args: &Map<String, Value>) -> Result<ToolOutput> {
        self.connect()?;
        let c = self.client.as_ref().expect("connected");
        let req = CallToolRequestParams::new(tool.to_string()).with_arguments(args.clone());
        let limit = Duration::from_secs(self.cfg.timeout_s.max(1));
        let answer = self
            .rt
            .block_on(async { tokio::time::timeout(limit, c.call_tool(req)).await });
        let r = match answer {
            Ok(r) => r.map_err(|e| Error::Mcp(format!("{tool}: {e}")))?,
            // No answer at all: say what CODESYS shows, if the bridge can tell.
            Err(_) => {
                return Err(match self.open_dialogs() {
                    Some(d) => Error::Dialog {
                        tool: tool.into(),
                        msg: format!("{DIALOG} {d} (and {tool} gave no answer within {limit:?})"),
                    },
                    None => Error::Mcp(format!("{tool}: no answer within {limit:?}")),
                });
            }
        };
        let text = r
            .content
            .iter()
            .filter_map(|c| c.as_text().map(|t| t.text.as_str()))
            .collect::<Vec<_>>()
            .join("\n");
        Ok(ToolOutput {
            text,
            is_error: r.is_error == Some(true),
        })
    }

    fn read_resource(&mut self, uri: &str) -> Result<ToolOutput> {
        self.connect()?;
        let c = self.client.as_ref().expect("connected");
        let limit = Duration::from_secs(self.cfg.timeout_s.max(1));
        let req = ReadResourceRequestParams::new(uri.to_string());
        let r = self.rt.block_on(async {
            tokio::time::timeout(limit, c.read_resource(req))
                .await
                .map_err(|_| Error::Mcp(format!("{uri}: no answer within {limit:?}")))?
                .map_err(|e| Error::Mcp(format!("resources/read {uri}: {e}")))
        })?;
        let text = r
            .contents
            .iter()
            .filter_map(|c| match c {
                ResourceContents::TextResourceContents { text, .. } => Some(text.as_str()),
                _ => None,
            })
            .collect::<Vec<_>>()
            .join("\n");
        Ok(ToolOutput {
            is_error: resource_failed(&text),
            text,
        })
    }
}

/// `GET /dialogs`' dialogs as `title: text [buttons]`, `; `-separated.
pub fn describe_dialogs(body: &Value) -> String {
    let one = |v: &Value| {
        v.as_str()
            .unwrap_or("")
            .split_whitespace()
            .collect::<Vec<_>>()
            .join(" ")
    };
    let strs = |v: &Value| -> Vec<String> {
        v.as_array()
            .into_iter()
            .flatten()
            .map(one)
            .filter(|s| !s.is_empty())
            .collect()
    };
    body["dialogs"]
        .as_array()
        .into_iter()
        .flatten()
        .map(|d| {
            let text = strs(&d["texts"]).join(" ");
            format!(
                "{}: {} [{}]",
                one(&d["title"]),
                if text.is_empty() { "(no text)" } else { &text },
                strs(&d["buttons"]).join(", ")
            )
        })
        .collect::<Vec<_>>()
        .join("; ")
}

/// The server answers a failed resource read with text, not an error (0.19.0
/// `project-structure`: "Error: ...", "Error retrieving structure.", "Could not parse structure
/// markers.", "Failed: ...").
fn resource_failed(text: &str) -> bool {
    let t = text.trim_start();
    ["Error", "Could not parse", "Failed:"]
        .iter()
        .any(|p| t.starts_with(p))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn failure_texts() {
        assert!(!ToolOutput::ok("Project saved").failed());
        assert!(ToolOutput::ok("Operation failed. Output:\n...").failed());
        assert!(ToolOutput::ok("Refused: declarationCode contains ...").failed());
        assert!(
            ToolOutput {
                text: "x".into(),
                is_error: true
            }
            .failed()
        );
    }

    #[test]
    fn failed_resource_reads() {
        for t in [
            "Error retrieving structure.\n\nx",
            "Could not parse structure markers.\n\nOutput:\nx",
            "Failed: timeout",
        ] {
            assert!(super::resource_failed(t), "{t}");
        }
        assert!(!super::resource_failed("- Unnamed (X)\n  - Device (X) {d}"));
    }

    #[test]
    fn short_keeps_the_bridge_notes() {
        let long = "x".repeat(3000);
        let o = ToolOutput::ok(format!(
            "Operation failed. Output:\n{long}\n[mcp-bridge] answered CODESYS dialog CODESYS: \
             No connection to the device. [OK] with OK\n[mcp-bridge] {DIALOG} CODESYS: Save? [Yes, No]"
        ));
        let s = o.short();
        assert!(s.len() < 2200, "{}", s.len());
        assert!(s.ends_with(&format!(
            "...\n[mcp-bridge] answered CODESYS dialog CODESYS: No connection to the device. [OK] \
             with OK\n[mcp-bridge] {DIALOG} CODESYS: Save? [Yes, No]"
        )));
        assert_eq!(
            dialog(&s).as_deref(),
            Some("CODESYS shows a dialog: CODESYS: Save? [Yes, No]")
        );
        assert_eq!(dialog("Operation failed. Output:\nCommand timed out"), None);
        // An open dialog that does not block (in its grace, a progress bar) is only a note.
        assert_eq!(
            dialog("Error: x\n[mcp-bridge] open, not blocking: CODESYS: Build [Cancel]"),
            None
        );
    }

    #[test]
    fn dialogs_of_the_bridge() {
        let body = serde_json::json!({"dialogs": [
            {"hwnd": 1, "title": "CODESYS", "texts": ["Do you want to\r\nsave?"], "buttons": ["Yes", "No"], "known": null},
            {"hwnd": 2, "title": "CODESYS", "texts": [], "buttons": ["OK"], "known": null}
        ], "answered": [], "pending": 1});
        assert_eq!(
            describe_dialogs(&body),
            "CODESYS: Do you want to save? [Yes, No]; CODESYS: (no text) [OK]"
        );
        assert_eq!(describe_dialogs(&serde_json::json!({"dialogs": []})), "");
    }

    #[test]
    fn only_a_refused_token_is_a_refusal() {
        let url = "http://127.0.0.1:40151/mcp";
        let not_up = format!(
            "{url}: Send message error Transport [..] error: Client error: error sending request \
             for url ({url}), when send initialize request"
        );
        assert!(!refused(&not_up, url));
        let url = "http://127.0.0.1:34011/mcp";
        assert!(!refused(
            &format!("{url}: error sending request for url ({url})"),
            url
        ));
        assert!(!refused(
            "http://h:401/mcp: error sending request",
            "http://h:401/mcp"
        ));
        assert!(refused(
            &format!("{url}: Unexpected server response: HTTP 401"),
            url
        ));
        assert!(refused(
            &format!("{url}: HTTP status client error (401 Unauthorized) for url ({url})"),
            url
        ));
        assert!(refused(
            &format!("{url}: x (cell.toml has no codesys_mcp.token_env; ...)"),
            url
        ));
    }

    #[test]
    fn short_drops_debug_lines() {
        let o = ToolOutput::ok("Operation failed. Output:\nDEBUG: a\n  DEBUG: b\nError in plug: x");
        assert_eq!(o.short(), "Operation failed. Output:\nError in plug: x");
    }
}
