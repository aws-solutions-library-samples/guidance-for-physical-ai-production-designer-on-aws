//! [`S3Store`]: an S3 bucket as a [`Store`], over plain HTTPS with SigV4 (`aws-sigv4`), so the
//! backend needs no AWS SDK.
//!
//! * Credentials: `AWS_ACCESS_KEY_ID` / `AWS_SECRET_ACCESS_KEY` (/ `AWS_SESSION_TOKEN`) if set, else
//!   the instance role's from IMDSv2 (refreshed before they expire). On a workspace that is its own
//!   role, which IAM limits to `users/<its FactoryosWorkspace tag>/*` and reading `catalogue/*`.
//! * Region: `--s3-region`, else `AWS_REGION` / `AWS_DEFAULT_REGION`, else the instance's.
//! * Presigned URLs ([`Store::presign`]) are signed with the same credentials, so they can do no more
//!   than the role, and never outlive [`super::PRESIGN_TTL`] (nor the credentials).

use std::time::{Duration, SystemTime, UNIX_EPOCH};

use anyhow::Context as _;
use aws_credential_types::Credentials;
use aws_sigv4::http_request::{
    PayloadChecksumKind, PercentEncodingMode, SignableBody, SignableRequest, SignatureLocation, SigningSettings, UriPathNormalizationMode, sign,
};
use aws_sigv4::sign::v4;
use aws_smithy_runtime_api::client::identity::Identity;
use axum::http;
use factoryos_protocol::storage::{Method, ObjectInfo, ObjectList, Scope, StorageMode, valid_prefix};
use futures::future::BoxFuture;
use tokio::sync::Mutex;

use super::{Fetched, MAX_SYNC_BYTES, ObjectKey, PutOutcome, Store, StorageOptions, UserId, scope_prefix};

const IMDS: &str = "http://169.254.169.254";
/// Credentials are refreshed when they have less than this left (more than a presigned URL's life).
const REFRESH_BEFORE: Duration = Duration::from_secs(15 * 60);
/// Most objects one listing answers with (pages of 1000).
const MAX_LIST: usize = 10_000;

pub struct S3Store {
    http: reqwest::Client,
    bucket: String,
    region: String,
    /// Normally empty; a prefix for everything (ends in `/`).
    root: String,
    /// `None`: AWS's virtual-hosted endpoint; else `<endpoint>/<bucket>/<key>` (path-style).
    endpoint: Option<String>,
    user: UserId,
    creds: Mutex<Option<Creds>>,
}

#[derive(Clone)]
struct Creds {
    access_key: String,
    secret_key: String,
    token: Option<String>,
    /// `None`: static (environment).
    expires: Option<SystemTime>,
}

impl S3Store {
    pub async fn open(options: &StorageOptions, user: UserId) -> anyhow::Result<Self> {
        let bucket = options.bucket.as_deref().map(str::trim).filter(|b| !b.is_empty()).context("--storage s3 needs --s3-bucket")?;
        anyhow::ensure!(valid_bucket(bucket), "--s3-bucket {bucket}: not a bucket name this store can use ([a-z0-9-], 3..63)");
        let root = options.root.as_deref().map(|r| r.trim().trim_matches('/')).unwrap_or("");
        anyhow::ensure!(root.is_empty() || factoryos_protocol::storage::valid_key(root), "--s3-root {root}: not a valid prefix");
        let root = if root.is_empty() { String::new() } else { format!("{root}/") };
        // rustls' provider, explicitly: with the whole workspace built, more than one is enabled.
        let _ = rustls::crypto::ring::default_provider().install_default();
        let http = reqwest::Client::builder().timeout(Duration::from_secs(60)).build()?;
        let region = match options.region.as_deref().map(str::trim).filter(|r| !r.is_empty()) {
            Some(region) => region.to_owned(),
            None => match env("AWS_REGION").or_else(|| env("AWS_DEFAULT_REGION")) {
                Some(region) => region,
                None => imds_get(&http, "/latest/meta-data/placement/region").await.context("the S3 region: set --s3-region")?,
            },
        };
        anyhow::ensure!(region.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'-'), "not a region: {region}");
        let endpoint = options.endpoint.as_deref().map(|e| e.trim().trim_end_matches('/').to_owned()).filter(|e| !e.is_empty());
        if let Some(endpoint) = &endpoint {
            anyhow::ensure!(endpoint.starts_with("https://") || endpoint.starts_with("http://127.0.0.1") || endpoint.starts_with("http://localhost"),
                "--s3-endpoint {endpoint}: HTTPS (plain HTTP on loopback only)");
        }
        let store = Self { http, bucket: bucket.to_owned(), region, root, endpoint, user, creds: Mutex::new(None) };
        // Fail at startup, not at the first save, when there are no credentials.
        store.credentials().await.context("S3 credentials")?;
        Ok(store)
    }

    pub fn describe(&self) -> String {
        format!("s3://{}/{} ({})", self.bucket, scope_prefix(&self.root, Scope::User, &self.user), self.region)
    }

    fn base(&self) -> String {
        match &self.endpoint {
            Some(endpoint) => format!("{endpoint}/{}", self.bucket),
            None => format!("https://{}.s3.{}.amazonaws.com", self.bucket, self.region),
        }
    }

    fn object_url(&self, key: &ObjectKey) -> String {
        format!("{}/{}", self.base(), encode_path(&key.object_key(&self.root, &self.user)))
    }

    async fn credentials(&self) -> anyhow::Result<Creds> {
        let mut cached = self.creds.lock().await;
        if let Some(creds) = cached.as_ref()
            && creds.expires.is_none_or(|at| at > SystemTime::now() + REFRESH_BEFORE)
        {
            return Ok(creds.clone());
        }
        let creds = match (env("AWS_ACCESS_KEY_ID"), env("AWS_SECRET_ACCESS_KEY")) {
            (Some(access_key), Some(secret_key)) => Creds { access_key, secret_key, token: env("AWS_SESSION_TOKEN"), expires: None },
            _ => imds_credentials(&self.http).await?,
        };
        *cached = Some(creds.clone());
        Ok(creds)
    }

    fn signing_settings(presign: Option<Duration>) -> SigningSettings {
        let mut settings = SigningSettings::default();
        // S3: the path is signed as sent (encoded once), never normalized.
        settings.percent_encoding_mode = PercentEncodingMode::Single;
        settings.uri_path_normalization_mode = UriPathNormalizationMode::Disabled;
        match presign {
            Some(ttl) => {
                settings.signature_location = SignatureLocation::QueryParams;
                settings.expires_in = Some(ttl);
                settings.payload_checksum_kind = PayloadChecksumKind::NoHeader;
            }
            None => settings.payload_checksum_kind = PayloadChecksumKind::XAmzSha256,
        }
        settings
    }

    /// `method url` signed (headers), with `body`.
    async fn send(&self, method: &str, url: &str, body: Vec<u8>) -> anyhow::Result<reqwest::Response> {
        self.send_with(method, url, &[], body).await
    }

    /// [`Self::send`] with more headers, signed too.
    async fn send_with(&self, method: &str, url: &str, extra: &[(&str, &str)], body: Vec<u8>) -> anyhow::Result<reqwest::Response> {
        let creds = self.credentials().await?;
        let identity: Identity = Credentials::new(&creds.access_key, &creds.secret_key, creds.token.clone(), creds.expires, "factoryos").into();
        let params = v4::SigningParams::builder()
            .identity(&identity)
            .region(&self.region)
            .name("s3")
            .time(SystemTime::now())
            .settings(Self::signing_settings(None))
            .build()?
            .into();
        let signable = SignableRequest::new(method, url, extra.iter().copied(), SignableBody::Bytes(&body))?;
        let (instructions, _) = sign(signable, &params)?.into_parts();
        let mut request = http::Request::builder().method(method).uri(url);
        for (name, value) in extra {
            request = request.header(*name, *value);
        }
        let mut request = request.body(())?;
        instructions.apply_to_request_http1x(&mut request);
        let method = reqwest::Method::from_bytes(method.as_bytes())?;
        let has_body = matches!(method, reqwest::Method::PUT | reqwest::Method::POST);
        let mut builder = self.http.request(method, url);
        for (name, value) in request.headers() {
            builder = builder.header(name.as_str(), value.as_bytes());
        }
        // S3 answers 411 Length Required to a PUT/POST without Content-Length, which an empty body
        // (e.g. a `.keep` or `.own` marker) gets sent without.
        if has_body {
            builder = builder.header(reqwest::header::CONTENT_LENGTH, body.len());
        }
        Ok(builder.body(body).send().await?)
    }

    async fn error(what: &str, response: reqwest::Response) -> anyhow::Error {
        let status = response.status();
        let body = response.text().await.unwrap_or_default();
        anyhow::anyhow!("S3 {what}: {status}: {}", body.chars().take(300).collect::<String>())
    }
}

impl Store for S3Store {
    fn mode(&self) -> StorageMode {
        StorageMode::S3
    }

    fn get<'a>(&'a self, key: &'a ObjectKey) -> BoxFuture<'a, anyhow::Result<Option<Vec<u8>>>> {
        Box::pin(async move {
            let response = self.send("GET", &self.object_url(key), Vec::new()).await?;
            match response.status().as_u16() {
                404 => Ok(None),
                200 => {
                    anyhow::ensure!(response.content_length().is_none_or(|n| n <= MAX_SYNC_BYTES), "{} is too large", key.key());
                    Ok(Some(response.bytes().await?.to_vec()))
                }
                _ => Err(Self::error(&format!("GET {}", key.key()), response).await),
            }
        })
    }

    fn head<'a>(&'a self, key: &'a ObjectKey) -> BoxFuture<'a, anyhow::Result<Option<ObjectInfo>>> {
        Box::pin(async move {
            let response = self.send("HEAD", &self.object_url(key), Vec::new()).await?;
            match response.status().as_u16() {
                404 => Ok(None),
                200 => Ok(Some(ObjectInfo {
                    key: key.key().to_owned(),
                    bytes: response.headers().get(reqwest::header::CONTENT_LENGTH).and_then(|v| v.to_str().ok()?.parse().ok()).unwrap_or(0),
                    etag: etag_header(&response),
                })),
                status => anyhow::bail!("S3 HEAD {}: {status}", key.key()),
            }
        })
    }

    fn get_if<'a>(&'a self, key: &'a ObjectKey, if_none_match: Option<&'a str>) -> BoxFuture<'a, anyhow::Result<Fetched>> {
        Box::pin(async move {
            let headers: Vec<(&str, &str)> = if_none_match.map(|etag| ("if-none-match", etag)).into_iter().collect();
            let response = self.send_with("GET", &self.object_url(key), &headers, Vec::new()).await?;
            match response.status().as_u16() {
                304 => Ok(Fetched::NotModified),
                404 => Ok(Fetched::Missing),
                200 => {
                    let etag = etag_header(&response);
                    anyhow::ensure!(response.content_length().is_none_or(|n| n <= MAX_SYNC_BYTES), "{} is too large", key.key());
                    Ok(Fetched::Found { bytes: response.bytes().await?.to_vec(), etag })
                }
                _ => Err(Self::error(&format!("GET {}", key.key()), response).await),
            }
        })
    }

    fn put_if<'a>(&'a self, key: &'a ObjectKey, bytes: Vec<u8>, if_match: Option<&'a str>) -> BoxFuture<'a, anyhow::Result<PutOutcome>> {
        Box::pin(async move {
            // S3 conditional writes: If-Match an ETag, or If-None-Match `*` for "not there yet".
            let header = match if_match {
                Some(etag) => ("if-match", etag),
                None => ("if-none-match", "*"),
            };
            let response = self.send_with("PUT", &self.object_url(key), &[header], bytes).await?;
            match response.status().as_u16() {
                // 412: not that version any more; 409: a concurrent conditional write.
                412 | 409 => Ok(PutOutcome::Conflict),
                s if (200..300).contains(&s) => Ok(PutOutcome::Written(etag_header(&response))),
                _ => Err(Self::error(&format!("PUT {}", key.key()), response).await),
            }
        })
    }

    // Who may write which scope is decided before a store is asked (storage::http, crate::authz); IAM
    // (the role's FactoryosCatalogueAdmin tag) decides it once more for the shared catalogue.
    fn put<'a>(&'a self, key: &'a ObjectKey, bytes: Vec<u8>) -> BoxFuture<'a, anyhow::Result<()>> {
        Box::pin(async move {
            let response = self.send_with("PUT", &self.object_url(key), logged_marker(key), bytes).await?;
            if response.status().is_success() { Ok(()) } else { Err(Self::error(&format!("PUT {}", key.key()), response).await) }
        })
    }

    fn delete<'a>(&'a self, key: &'a ObjectKey) -> BoxFuture<'a, anyhow::Result<()>> {
        Box::pin(async move {
            let response = self.send("DELETE", &self.object_url(key), Vec::new()).await?;
            if response.status().is_success() || response.status() == 404 {
                Ok(())
            } else {
                Err(Self::error(&format!("DELETE {}", key.key()), response).await)
            }
        })
    }

    fn copy<'a>(&'a self, from: &'a ObjectKey, to: &'a ObjectKey) -> BoxFuture<'a, anyhow::Result<()>> {
        Box::pin(async move {
            if from == to {
                return Ok(());
            }
            // CopyObject: the source as `/<bucket>/<key>`, URL-encoded.
            let source = format!("/{}/{}", self.bucket, encode_path(&from.object_key(&self.root, &self.user)));
            let mut headers = vec![("x-amz-copy-source", source.as_str())];
            if !logged_marker(to).is_empty() {
                // The copy's own metadata (the marker), not the source's.
                headers.push(("x-amz-metadata-directive", "REPLACE"));
                headers.extend_from_slice(logged_marker(to));
            }
            let response = self.send_with("PUT", &self.object_url(to), &headers, Vec::new()).await?;
            let status = response.status();
            let body = response.text().await.unwrap_or_default();
            // S3 can answer 200 with an error in the body.
            if status.is_success() && !body.contains("<Error>") {
                Ok(())
            } else {
                anyhow::bail!("S3 copy {} → {}: {status}: {}", from.key(), to.key(), body.chars().take(300).collect::<String>())
            }
        })
    }

    fn list<'a>(&'a self, scope: Scope, prefix: &'a str) -> BoxFuture<'a, anyhow::Result<ObjectList>> {
        Box::pin(async move {
            anyhow::ensure!(valid_prefix(prefix), "not a valid prefix: {prefix:?}");
            let base = scope_prefix(&self.root, scope, &self.user);
            let full = format!("{base}{prefix}");
            let mut list = ObjectList::default();
            let mut token: Option<String> = None;
            loop {
                let mut url = format!("{}/?list-type=2&prefix={}", self.base(), encode_query(&full));
                if let Some(token) = &token {
                    url.push_str(&format!("&continuation-token={}", encode_query(token)));
                }
                let response = self.send("GET", &url, Vec::new()).await?;
                if !response.status().is_success() {
                    return Err(Self::error("list", response).await);
                }
                let page = parse_list(&response.text().await?);
                for (key, bytes, etag) in page.objects {
                    // Only keys of this scope, relative to it (S3 answers only the prefix asked for).
                    if let Some(rel) = key.strip_prefix(&base) {
                        list.objects.push(ObjectInfo { key: rel.to_owned(), bytes, etag });
                    }
                }
                if list.objects.len() >= MAX_LIST {
                    list.truncated = page.next.is_some() || list.objects.len() > MAX_LIST;
                    list.objects.truncate(MAX_LIST);
                    break;
                }
                match page.next {
                    Some(next) => token = Some(next),
                    None => break,
                }
            }
            Ok(list)
        })
    }

    fn presign<'a>(&'a self, key: &'a ObjectKey, method: Method, ttl: Duration) -> BoxFuture<'a, anyhow::Result<Option<(String, SystemTime)>>> {
        Box::pin(async move {
            let creds = self.credentials().await?;
            let now = SystemTime::now();
            // Not past the credentials' own expiry: the URL would stop working early.
            let ttl = creds.expires.and_then(|at| at.duration_since(now).ok()).map_or(ttl, |left| ttl.min(left));
            let identity: Identity = Credentials::new(&creds.access_key, &creds.secret_key, creds.token.clone(), creds.expires, "factoryos").into();
            let params = v4::SigningParams::builder()
                .identity(&identity)
                .region(&self.region)
                .name("s3")
                .time(now)
                .settings(Self::signing_settings(Some(ttl)))
                .build()?
                .into();
            let method = match method {
                Method::Get => "GET",
                Method::Put => "PUT",
            };
            let url = self.object_url(key);
            let signable = SignableRequest::new(method, &url, std::iter::empty(), SignableBody::UnsignedPayload)?;
            let (instructions, _) = sign(signable, &params)?.into_parts();
            let mut request = http::Request::builder().method(method).uri(&url).body(())?;
            instructions.apply_to_request_http1x(&mut request);
            Ok(Some((request.uri().to_string(), now + ttl)))
        })
    }
}

fn env(name: &str) -> Option<String> {
    std::env::var(name).ok().map(|v| v.trim().to_owned()).filter(|v| !v.is_empty())
}

/// Lowercase letters, digits and `-` (no dots: they break the virtual-hosted endpoint's TLS).
fn valid_bucket(name: &str) -> bool {
    (3..=63).contains(&name.len())
        && name.bytes().all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || b == b'-')
        && !name.starts_with('-')
        && !name.ends_with('-')
}

/// RFC 3986 encoding of an object key for the URL path (`/` kept).
fn encode_path(key: &str) -> String {
    encode(key, true)
}

fn encode_query(value: &str) -> String {
    encode(value, false)
}

fn encode(value: &str, keep_slash: bool) -> String {
    let mut out = String::with_capacity(value.len());
    for b in value.bytes() {
        if b.is_ascii_alphanumeric() || matches!(b, b'-' | b'_' | b'.' | b'~') || (keep_slash && b == b'/') {
            out.push(b as char);
        } else {
            out.push_str(&format!("%{b:02X}"));
        }
    }
    out
}

/// The metadata a backend's write of the shared catalogue carries: the backend logs the change in
/// `catalogue/.revision` itself (storage::sync), so the change-log Lambda for writes from outside
/// Factory OS (infra/lambda/catalogue-change-log) skips it. Keep the name in step with that Lambda.
pub const LOGGED_HEADER: &str = "x-amz-meta-factoryos-logged";

fn logged_marker(key: &ObjectKey) -> &'static [(&'static str, &'static str)] {
    if key.scope == Scope::Catalogue { &[(LOGGED_HEADER, "1")] } else { &[] }
}

/// An answer's `ETag`, as S3 sends it (quoted).
fn etag_header(response: &reqwest::Response) -> Option<String> {
    response.headers().get(reqwest::header::ETAG).and_then(|v| v.to_str().ok()).map(str::to_owned)
}

struct ListPage {
    /// Key, size, ETag.
    objects: Vec<(String, u64, Option<String>)>,
    next: Option<String>,
}

/// The keys, sizes and continuation token of a `ListObjectsV2` answer.
fn parse_list(xml: &str) -> ListPage {
    let mut objects = Vec::new();
    let mut rest = xml;
    while let Some(start) = rest.find("<Contents>") {
        let after = &rest[start + "<Contents>".len()..];
        let Some(end) = after.find("</Contents>") else { break };
        let contents = &after[..end];
        if let Some(key) = element(contents, "Key") {
            let size = element(contents, "Size").and_then(|s| s.parse().ok()).unwrap_or(0);
            objects.push((unescape(key), size, element(contents, "ETag").map(unescape)));
        }
        rest = &after[end..];
    }
    let truncated = element(xml, "IsTruncated") == Some("true");
    let next = if truncated { element(xml, "NextContinuationToken").map(unescape) } else { None };
    ListPage { objects, next }
}

fn element<'a>(xml: &'a str, name: &str) -> Option<&'a str> {
    let open = format!("<{name}>");
    let start = xml.find(&open)? + open.len();
    let end = xml[start..].find(&format!("</{name}>"))?;
    Some(&xml[start..start + end])
}

fn unescape(text: &str) -> String {
    text.replace("&lt;", "<").replace("&gt;", ">").replace("&quot;", "\"").replace("&apos;", "'").replace("&#13;", "\r").replace("&amp;", "&")
}

/// IMDSv2: a session token, then `path`.
async fn imds_get(http: &reqwest::Client, path: &str) -> anyhow::Result<String> {
    let token = http
        .put(format!("{IMDS}/latest/api/token"))
        .header("X-aws-ec2-metadata-token-ttl-seconds", "300")
        .timeout(Duration::from_secs(2))
        .send()
        .await
        .context("IMDS (not on EC2?)")?
        .error_for_status()?
        .text()
        .await?;
    let answer = http
        .get(format!("{IMDS}{path}"))
        .header("X-aws-ec2-metadata-token", token)
        .timeout(Duration::from_secs(2))
        .send()
        .await?
        .error_for_status()?
        .text()
        .await?;
    Ok(answer.trim().to_owned())
}

async fn imds_credentials(http: &reqwest::Client) -> anyhow::Result<Creds> {
    let role = imds_get(http, "/latest/meta-data/iam/security-credentials/").await?;
    let role = role.lines().next().unwrap_or("").trim();
    anyhow::ensure!(!role.is_empty() && role.bytes().all(|b| b.is_ascii_alphanumeric() || b"+=,.@_-".contains(&b)), "no instance role");
    let json: serde_json::Value = serde_json::from_str(&imds_get(http, &format!("/latest/meta-data/iam/security-credentials/{role}")).await?)?;
    let field = |name: &str| json.get(name).and_then(|v| v.as_str()).map(str::to_owned).with_context(|| format!("instance credentials without {name}"));
    Ok(Creds {
        access_key: field("AccessKeyId")?,
        secret_key: field("SecretAccessKey")?,
        token: Some(field("Token")?),
        expires: Some(parse_rfc3339(&field("Expiration")?).context("instance credentials: Expiration")?),
    })
}

/// `YYYY-MM-DDTHH:MM:SSZ` (IMDS's form; fractions ignored) as a time.
fn parse_rfc3339(text: &str) -> Option<SystemTime> {
    let b = text.as_bytes();
    if b.len() < 20 || b[4] != b'-' || b[7] != b'-' || b[10] != b'T' || b[13] != b':' || b[16] != b':' || !text.ends_with('Z') {
        return None;
    }
    let num = |range: std::ops::Range<usize>| text.get(range)?.parse::<i64>().ok();
    let (y, m, d) = (num(0..4)?, num(5..7)?, num(8..10)?);
    let (hh, mm, ss) = (num(11..13)?, num(14..16)?, num(17..19)?);
    if !(1..=12).contains(&m) || !(1..=31).contains(&d) || hh > 23 || mm > 59 || ss > 60 {
        return None;
    }
    // Days from the civil date (Howard Hinnant's algorithm).
    let y = if m <= 2 { y - 1 } else { y };
    let era = y.div_euclid(400);
    let yoe = y - era * 400;
    let mp = (m + 9) % 12;
    let doy = (153 * mp + 2) / 5 + d - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    let days = era * 146_097 + doe - 719_468;
    let secs = days * 86_400 + hh * 3600 + mm * 60 + ss;
    Some(UNIX_EPOCH + Duration::from_secs(u64::try_from(secs).ok()?))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn store(endpoint: Option<&str>) -> S3Store {
        S3Store {
            http: reqwest::Client::new(),
            bucket: "examplebucket".into(),
            region: "us-east-1".into(),
            root: String::new(),
            endpoint: endpoint.map(str::to_owned),
            user: UserId::new("jdoe").unwrap(),
            creds: Mutex::new(Some(Creds {
                access_key: "AKIAIOSFODNN7EXAMPLE".into(),
                secret_key: "wJalrXUtnFEMI/K7MDENG/bPxRfiCYEXAMPLEKEY".into(),
                token: None,
                expires: None,
            })),
        }
    }

    #[tokio::test]
    async fn presigned_urls_are_scoped_to_the_user() {
        let s3 = store(None);
        let key = ObjectKey::new(Scope::User, "products/a b/product.json").unwrap();
        let (url, expires) = s3.presign(&key, Method::Get, Duration::from_secs(300)).await.unwrap().unwrap();
        assert!(url.starts_with("https://examplebucket.s3.us-east-1.amazonaws.com/users/jdoe/products/a%20b/product.json?"), "{url}");
        for param in ["X-Amz-Algorithm=AWS4-HMAC-SHA256", "X-Amz-Expires=300", "X-Amz-SignedHeaders=host", "X-Amz-Signature="] {
            assert!(url.contains(param), "{param} in {url}");
        }
        assert!(expires > SystemTime::now() + Duration::from_secs(290));
        // The shared catalogue (who may PUT is storage::http's to decide).
        let cat = ObjectKey::new(Scope::Catalogue, "x.urdf").unwrap();
        let (url, _) = s3.presign(&cat, Method::Get, Duration::from_secs(60)).await.unwrap().unwrap();
        assert!(url.contains("/catalogue/x.urdf?"), "{url}");
        let (url, _) = s3.presign(&cat, Method::Put, Duration::from_secs(60)).await.unwrap().unwrap();
        assert!(url.starts_with("https://examplebucket.s3.us-east-1.amazonaws.com/catalogue/x.urdf?"), "{url}");
    }

    /// The presigned GET example of the S3 SigV4 documentation ("Authenticating Requests: Using
    /// Query Parameters"), byte for byte: the signing is S3's.
    #[test]
    fn signs_like_the_s3_documentation() {
        let identity: Identity =
            Credentials::new("AKIAIOSFODNN7EXAMPLE", "wJalrXUtnFEMI/K7MDENG/bPxRfiCYEXAMPLEKEY", None, None, "test").into();
        let time = UNIX_EPOCH + Duration::from_secs(1_369_353_600); // 2013-05-24T00:00:00Z
        let params = v4::SigningParams::builder()
            .identity(&identity)
            .region("us-east-1")
            .name("s3")
            .time(time)
            .settings(S3Store::signing_settings(Some(Duration::from_secs(86_400))))
            .build()
            .unwrap()
            .into();
        let url = "https://examplebucket.s3.amazonaws.com/test.txt";
        let signable = SignableRequest::new("GET", url, std::iter::empty(), SignableBody::UnsignedPayload).unwrap();
        let (_, signature) = sign(signable, &params).unwrap().into_parts();
        assert_eq!(signature, "aeeed9bbccd4d02ee5c0109b86d86835f995330da4c265957d157751f604d404");
    }

    #[test]
    fn catalogue_writes_carry_the_logged_marker() {
        let cat = ObjectKey::new(Scope::Catalogue, "a.urdf").unwrap();
        assert_eq!(logged_marker(&cat), &[("x-amz-meta-factoryos-logged", "1")]);
        assert!(logged_marker(&ObjectKey::new(Scope::User, "catalogue/a.urdf").unwrap()).is_empty());
    }

    #[test]
    fn helpers() {
        assert_eq!(encode_path("a b/c+d/é"), "a%20b/c%2Bd/%C3%A9");
        assert!(valid_bucket("factoryos-data-123456789012") && !valid_bucket("a.b") && !valid_bucket("Ab") && !valid_bucket("-ab"));
        assert_eq!(parse_rfc3339("2013-05-24T00:00:00Z"), Some(UNIX_EPOCH + Duration::from_secs(1_369_353_600)));
        assert_eq!(parse_rfc3339("garbage"), None);
        let xml = "<ListBucketResult><IsTruncated>true</IsTruncated><Contents><Key>users/jdoe/a&amp;b</Key><ETag>&quot;e1&quot;</ETag><Size>3</Size></Contents>\
                   <Contents><Key>users/jdoe/c</Key><Size>0</Size></Contents><NextContinuationToken>t+1</NextContinuationToken></ListBucketResult>";
        let page = parse_list(xml);
        assert_eq!(page.objects, vec![("users/jdoe/a&b".to_owned(), 3, Some("\"e1\"".to_owned())), ("users/jdoe/c".to_owned(), 0, None)]);
        assert_eq!(page.next.as_deref(), Some("t+1"));
        assert_eq!(store(Some("http://127.0.0.1:9000")).base(), "http://127.0.0.1:9000/examplebucket");
    }
}
