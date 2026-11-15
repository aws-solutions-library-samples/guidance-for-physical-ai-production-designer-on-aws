//! The MCP side: [`Facade`] as an rmcp server handler (the only code that uses rmcp's handler API),
//! and the transport wrapper that notes whether a client used the session.

use std::convert::Infallible;
use std::future::Future;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use bytes::Bytes;
use rmcp::model::{
    CallToolRequestParams, CallToolResponse, JsonRpcMessage, ListToolsResult,
    PaginatedRequestParams, ServerConfig,
};
use rmcp::service::{RequestContext, RxJsonRpcMessage, TxJsonRpcMessage};
use rmcp::transport::Transport;
use rmcp::{ErrorData, RoleServer, ServerHandler};
use serde_json::Value;

use crate::{Body, Facade};

impl<S, B> ServerHandler for Facade<S>
where
    S: tower::Service<http::Request<Body>, Response = http::Response<B>, Error = Infallible>
        + Clone
        + Send
        + Sync
        + 'static,
    S::Future: Send + 'static,
    B: http_body::Body<Data = Bytes> + Send + 'static,
    B::Error: std::fmt::Display,
{
    fn get_info(&self) -> ServerConfig {
        self.inner.info.clone()
    }

    /// Every tool in one page; a cursor is ignored.
    async fn list_tools(
        &self,
        _request: Option<PaginatedRequestParams>,
        _context: RequestContext<RoleServer>,
    ) -> Result<ListToolsResult, ErrorData> {
        Ok(ListToolsResult::with_all_items(self.inner.tools.clone()))
    }

    async fn call_tool(
        &self,
        request: CallToolRequestParams,
        _context: RequestContext<RoleServer>,
    ) -> Result<CallToolResponse, ErrorData> {
        let arguments = request.arguments.map_or(Value::Null, Value::Object);
        self.call(&request.name, arguments)
            .await
            .map(CallToolResponse::from)
    }
}

/// A server transport that sets `used` once the client has sent a request.
pub(crate) struct Watched<T> {
    pub(crate) inner: T,
    pub(crate) used: Arc<AtomicBool>,
}

impl<T: Transport<RoleServer>> Transport<RoleServer> for Watched<T> {
    type Error = T::Error;

    fn send(
        &mut self,
        item: TxJsonRpcMessage<RoleServer>,
    ) -> impl Future<Output = Result<(), Self::Error>> + Send + 'static {
        self.inner.send(item)
    }

    async fn receive(&mut self) -> Option<RxJsonRpcMessage<RoleServer>> {
        let message = self.inner.receive().await;
        if let Some(JsonRpcMessage::Request(_)) = message {
            self.used.store(true, Ordering::Relaxed);
        }
        message
    }

    fn close(&mut self) -> impl Future<Output = Result<(), Self::Error>> + Send {
        self.inner.close()
    }
}
