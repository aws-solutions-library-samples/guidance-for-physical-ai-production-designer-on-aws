// FactoryOS public deployment: the CloudFront viewer-request gate (Lambda@Edge).
//
// Every request to the distribution passes through here. A request reaches the instance only with a valid session of
// the stack's own Cognito user pool: the pool's ID and access tokens (RS256, verified against the pool's JWKS: issuer,
// client, token_use, expiry) in `__Host-` cookies (HttpOnly, Secure, SameSite=Lax), refreshed here with the refresh
// token when they run out. The login is the pool's managed login (authorization code flow with PKCE, state and nonce)
// through /_auth/*. Without a session a page load is sent to the login (302), anything else (the API, HEAD, POST,
// WebSocket upgrades) gets 401 with a WWW-Authenticate header of this gate (realm="factoryos").
//
// Header hygiene: the identity headers the backend trusts are deleted from every request first, then set only for a
// verified session (`X-User-Alias: default`: the public sample has one workspace, shared by every pool user).
// WebSocket upgrades and state-changing requests from a foreign Origin are refused.
//
// The origin secret (deployment/CONTRACT.md "Origin secret"): every request forwarded to the origin carries
// `x-origin-verify: <secret>`, which the backend requires from every peer that is not loopback; a client-sent
// x-origin-verify is deleted first like the identity headers. The secret is never logged or put into a response.
//
// Lambda@Edge has no environment variables: the name of the SSM parameter (us-east-1) holding the pool, client and
// domains is in settings.json, written next to this file when the CDK app bundles it; the secret is the SecureString
// next to it, `/factoryos/<stack>/edge-origin-secret` (settings.json `secretParameterName`, else the configuration
// parameter's name with `edge-config` replaced by `edge-origin-secret`), written by the installer or the stack's
// OriginSecret custom resource, read with decryption (the AWS managed key aws/ssm). No dependencies besides the
// Lambda runtime's AWS SDK, which the configuration load imports.
//
// That load runs at INIT (createHandler's `preload`), never on a request: a viewer-request function gets 128 MB and
// 5 s, and loading the SDK off the runtime at 128 MB took ~4.7 s of it, so cold starts timed out and CloudFront
// answered 503. INIT has its own budget (~10 s) and is not charged against the 5 s.

import crypto from 'node:crypto';
import { readFileSync } from 'node:fs';

export const COOKIES = Object.freeze({
  id: '__Host-fos_id',
  access: '__Host-fos_access',
  refresh: '__Host-fos_refresh',
  login: '__Host-fos_login',
});
export const AUTH_PREFIX = '/_auth/';
export const CALLBACK_PATH = '/_auth/callback';
export const LOGIN_PATH = '/_auth/login';
export const LOGOUT_PATH = '/_auth/logout';
export const REALM = 'factoryos';
export const WWW_AUTHENTICATE = `FactoryOS realm="${REALM}", login="${LOGIN_PATH}"`;

/** Client-sent headers deleted from every request (the backend's identity headers, the origin's secrets). */
export const STRIPPED_HEADERS = Object.freeze([
  'x-user-alias',
  'x-workspace',
  'x-terminal-principal',
  'x-isaac-principal',
  'x-smartpad-site',
  'x-forwarded-host',
  'x-origin-verify',
  'x-audit-sub',
]);
export const STRIPPED_PREFIXES = Object.freeze(['x-amzn-']);

const CONFIG_TTL_MS = 5 * 60 * 1000;
const CONFIG_TIMEOUT_MS = 4000;
const JWKS_REFETCH_MS = 60 * 1000;
const HTTP_TIMEOUT_MS = 3000;
const LOGIN_COOKIE_TTL_S = 600;
const CLOCK_SKEW_S = 300;
// A token that runs out within this many seconds counts as run out (refreshed now rather than mid-request).
const EXPIRY_MARGIN_S = 30;
const MAX_TOKEN_LENGTH = 8192;
const B64URL = /^[A-Za-z0-9_-]+$/;
/** The origin secret: base64url, 32 to 256 characters (the installer's are 48 random bytes, 64 characters). */
export const ORIGIN_SECRET = /^[A-Za-z0-9_-]{32,256}$/;
export const ORIGIN_HEADER = 'x-origin-verify';

// ---------------------------------------------------------------------------------------------------------------
// Configuration

/** The configuration the stack writes (lib/factoryos-stack.ts, EdgeConfig), validated. */
export function parseConfig(text) {
  const c = JSON.parse(text);
  const ok = c && c.format === 1
    && /^[a-z]{2}(-[a-z]+)+-\d$/.test(c.region)
    && /^[a-z]{2}(-[a-z]+)+-\d_[0-9A-Za-z]+$/.test(c.userPoolId)
    && /^[0-9a-z]{16,128}$/.test(c.clientId)
    && /^[a-z0-9.-]+$/.test(c.cognitoDomain)
    && /^[a-z0-9.-]+$/.test(c.distributionDomain)
    && Number.isInteger(c.refreshTtlSeconds) && c.refreshTtlSeconds > 0
    // Required: without it the backend refuses every forwarded request, so the gate refuses (503) instead.
    && typeof c.originSecret === 'string' && ORIGIN_SECRET.test(c.originSecret); // pragma: allowlist secret (the field's name; the value is checked, never logged)
  if (!ok) throw new Error('edge configuration is invalid');
  return Object.freeze({ ...c, issuer: `https://cognito-idp.${c.region}.amazonaws.com/${c.userPoolId}` });
}

/** The SecureString holding the origin secret, next to the configuration parameter. */
export function secretParameterName(settings) {
  if (typeof settings.secretParameterName === 'string' && settings.secretParameterName) return settings.secretParameterName; // pragma: allowlist secret (the setting's name)
  if (!/\/edge-config$/.test(settings.parameterName)) throw new Error('settings.json names no origin secret parameter');
  return settings.parameterName.replace(/\/edge-config$/, '/edge-origin-secret');
}

/** The configuration JSON with the origin secret added (`originSecret`: the SecureString's value wins). */
export function withOriginSecret(configText, secret) {
  return JSON.stringify({ ...JSON.parse(configText), originSecret: String(secret ?? '').trim() });
}

async function loadConfigFromSsm() {
  const settings = JSON.parse(readFileSync(new URL('./settings.json', import.meta.url), 'utf8'));
  const { SSMClient, GetParameterCommand } = await import('@aws-sdk/client-ssm');
  const ssm = new SSMClient({ region: settings.parameterRegion });
  // Two GetParameter calls (the role's only SSM action), not GetParameters, in parallel. Bounded: a hanging call would
  // otherwise sit there for the rest of INIT's budget and fail every request after it.
  const bounded = () => ({ abortSignal: AbortSignal.timeout(CONFIG_TIMEOUT_MS) });
  const [config, secret] = await Promise.all([
    ssm.send(new GetParameterCommand({ Name: settings.parameterName }), bounded()),
    ssm.send(new GetParameterCommand({ Name: secretParameterName(settings), WithDecryption: true }), bounded()),
  ]);
  return withOriginSecret(config.Parameter.Value, secret.Parameter.Value);
}

// ---------------------------------------------------------------------------------------------------------------
// Small helpers

function b64urlJson(part) {
  if (typeof part !== 'string' || !B64URL.test(part)) throw new Error('malformed token');
  return JSON.parse(Buffer.from(part, 'base64url').toString('utf8'));
}

function randomToken(random, bytes = 32) {
  return random(bytes).toString('base64url');
}

function safeEqual(a, b) {
  if (typeof a !== 'string' || typeof b !== 'string') return false;
  const x = Buffer.from(a);
  const y = Buffer.from(b);
  return x.length === y.length && crypto.timingSafeEqual(x, y);
}

function header(request, name) {
  return request.headers?.[name]?.[0]?.value;
}

export function parseCookies(request) {
  const out = {};
  for (const h of request.headers?.cookie || []) {
    for (const pair of String(h.value).split(';')) {
      const eq = pair.indexOf('=');
      if (eq < 0) continue;
      const name = pair.slice(0, eq).trim();
      if (name && !(name in out)) out[name] = pair.slice(eq + 1).trim();
    }
  }
  return out;
}

/** A `__Host-` cookie: always Path=/, Secure, HttpOnly, SameSite=Lax, never a Domain. */
export function setCookie(name, value, maxAge) {
  return `${name}=${value}; Path=/; Secure; HttpOnly; SameSite=Lax; Max-Age=${Math.max(0, Math.floor(maxAge))}`;
}

function clearCookies(names) {
  return names.map((n) => setCookie(n, '', 0));
}

const SESSION_COOKIES = [COOKIES.id, COOKIES.access, COOKIES.refresh];

function baseHeaders(extra = {}) {
  const h = {
    'cache-control': [{ key: 'Cache-Control', value: 'no-store' }],
    'x-content-type-options': [{ key: 'X-Content-Type-Options', value: 'nosniff' }],
    'strict-transport-security': [{ key: 'Strict-Transport-Security', value: 'max-age=63072000; includeSubDomains' }],
    'referrer-policy': [{ key: 'Referrer-Policy', value: 'no-referrer' }],
  };
  for (const [k, v] of Object.entries(extra)) h[k] = v;
  return h;
}

function withCookies(headers, cookies) {
  if (cookies.length) headers['set-cookie'] = cookies.map((value) => ({ key: 'Set-Cookie', value }));
  return headers;
}

const DESCRIPTIONS = { 302: 'Found', 307: 'Temporary Redirect', 400: 'Bad Request', 401: 'Unauthorized', 403: 'Forbidden', 404: 'Not Found', 503: 'Service Unavailable' };

function respond(status, body, { contentType = 'text/plain; charset=utf-8', cookies = [], extra = {} } = {}) {
  const headers = baseHeaders({ 'content-type': [{ key: 'Content-Type', value: contentType }], ...extra });
  return { status: String(status), statusDescription: DESCRIPTIONS[status] || 'Error', headers: withCookies(headers, cookies), body };
}

function redirect(status, location, cookies = []) {
  const headers = baseHeaders({ location: [{ key: 'Location', value: location }] });
  return { status: String(status), statusDescription: DESCRIPTIONS[status], headers: withCookies(headers, cookies) };
}

function unauthorized(cookies = []) {
  return respond(401, JSON.stringify({
    error: 'unauthorized',
    message: 'Not signed in or the session has expired: reload the page to sign in.',
    login: LOGIN_PATH,
  }), {
    contentType: 'application/json',
    cookies,
    extra: { 'www-authenticate': [{ key: 'WWW-Authenticate', value: WWW_AUTHENTICATE }] },
  });
}

function forbidden(reason) {
  return respond(403, `Forbidden: ${reason}\n`, { extra: { 'x-factoryos-gate': [{ key: 'X-FactoryOS-Gate', value: 'denied' }] } });
}

/** A WebSocket upgrade. CloudFront does not show Lambda@Edge `Upgrade` / `Connection`; the Sec-WebSocket-* headers it does. */
export function isUpgrade(request) {
  const h = request.headers || {};
  return Boolean(h['sec-websocket-key'] || h['sec-websocket-version'] || h.upgrade);
}

/** A page load (sign in with a redirect), not a running page's fetch / XHR / WebSocket (those get 401). */
export function isNavigation(request) {
  if ((request.method || 'GET') !== 'GET' || isUpgrade(request)) return false;
  const mode = header(request, 'sec-fetch-mode');
  if (mode) return mode === 'navigate';
  if (request.uri.startsWith('/api/')) return false;
  const accept = header(request, 'accept') || '';
  return accept === '' || accept.includes('text/html');
}

/** A path on this site to come back to after the login (never another host, never /_auth/ itself). */
export function safeReturnPath(path) {
  if (typeof path !== 'string' || path.length > 2048) return '/';
  if (!path.startsWith('/') || path.startsWith('//') || path.startsWith('/\\') || path.startsWith(AUTH_PREFIX)) return '/';
  if (/[\u0000-\u001f\u007f]/.test(path)) return '/';
  return path;
}

/** Deletes the client-sent identity headers (STRIPPED_HEADERS, x-amzn-*). */
export function stripHeaders(request) {
  for (const name of Object.keys(request.headers || {})) {
    const n = name.toLowerCase();
    if (STRIPPED_HEADERS.includes(n) || STRIPPED_PREFIXES.some((p) => n.startsWith(p))) delete request.headers[name];
  }
}

/** The request's Cookie header without this gate's cookies (the backend never sees the tokens). */
function dropOwnCookies(request) {
  const own = new Set(Object.values(COOKIES));
  const kept = [];
  for (const h of request.headers.cookie || []) {
    for (const pair of String(h.value).split(';')) {
      const name = pair.slice(0, Math.max(0, pair.indexOf('='))).trim();
      if (pair.trim() && !own.has(name)) kept.push(pair.trim());
    }
  }
  if (kept.length) request.headers.cookie = [{ key: 'Cookie', value: kept.join('; ') }];
  else delete request.headers.cookie;
}

// ---------------------------------------------------------------------------------------------------------------
// The gate

export function createHandler({
  loadConfig = loadConfigFromSsm,
  fetchImpl = (...a) => fetch(...a),
  now = () => Date.now(),
  random = crypto.randomBytes,
  log = (entry) => console.log(JSON.stringify(entry)),
  // Load the configuration at INIT (see the file's header), in the Lambda runtime only: importing this module must
  // stay free of side effects everywhere else (the tests import it and inject `loadConfig`).
  preload = Boolean(process.env.AWS_LAMBDA_FUNCTION_NAME),
} = {}) {
  let config = null;
  let configAt = 0;
  let loading = null;
  let jwks = { issuer: '', keys: new Map(), at: 0 };

  /** The load, shared by everything that needs the configuration while one is in flight (INIT and requests alike). */
  function loadOnce() {
    loading ||= (async () => {
      try {
        const fresh = parseConfig(await loadConfig());
        config = fresh;
        configAt = now();
      } catch (e) {
        // A stale configuration beats none (SSM throttling); without one every request is refused.
        if (!config) throw e;
        log({ gate: 'config-reload-failed', error: String(e?.message || e) });
      } finally {
        // Cleared whatever happened: a load that failed is tried again by the next request.
        loading = null;
      }
      return config;
    })();
    return loading;
  }

  async function getConfig() {
    if (config && now() - configAt < CONFIG_TTL_MS) return config;
    return loadOnce();
  }

  // INIT. `ready` never rejects (a rejection would fail INIT itself): after a failed preload the first request loads
  // again and answers 503 on its own, as it does without a preload. The module awaits it (its last line).
  const ready = preload ? loadOnce().then(() => {}, () => {}) : Promise.resolve();

  async function fetchWithTimeout(url, init = {}) {
    return fetchImpl(url, { ...init, signal: AbortSignal.timeout(HTTP_TIMEOUT_MS) });
  }

  async function signingKey(cfg, kid) {
    if (jwks.issuer !== cfg.issuer) jwks = { issuer: cfg.issuer, keys: new Map(), at: 0 };
    if (!jwks.keys.has(kid) && now() - jwks.at > JWKS_REFETCH_MS) {
      jwks.at = now();
      const r = await fetchWithTimeout(`${cfg.issuer}/.well-known/jwks.json`);
      if (!r.ok) throw new Error(`JWKS ${r.status}`);
      const body = await r.json();
      jwks.keys = new Map((body.keys || []).filter((k) => typeof k.kid === 'string').map((k) => [k.kid, k]));
    }
    return jwks.keys.get(kid) || null;
  }

  /** A Cognito JWT of this pool and client, `tokenUse` 'id' or 'access': its claims, or an Error thrown. */
  async function verifyToken(token, tokenUse, cfg) {
    if (typeof token !== 'string' || token.length > MAX_TOKEN_LENGTH) throw new Error('no token');
    const parts = token.split('.');
    if (parts.length !== 3 || !parts.every((p) => B64URL.test(p))) throw new Error('malformed token');
    const head = b64urlJson(parts[0]);
    if (head.alg !== 'RS256') throw new Error('algorithm not allowed');
    if (typeof head.kid !== 'string') throw new Error('no key id');
    const jwk = await signingKey(cfg, head.kid);
    if (!jwk || jwk.kty !== 'RSA' || (jwk.use && jwk.use !== 'sig') || (jwk.alg && jwk.alg !== 'RS256')) {
      throw new Error('unknown key');
    }
    const key = crypto.createPublicKey({ key: jwk, format: 'jwk' });
    const valid = crypto.verify('RSA-SHA256', Buffer.from(`${parts[0]}.${parts[1]}`), key, Buffer.from(parts[2], 'base64url'));
    if (!valid) throw new Error('bad signature');
    const claims = b64urlJson(parts[1]);
    const t = now() / 1000;
    if (claims.iss !== cfg.issuer) throw new Error('wrong issuer');
    if (claims.token_use !== tokenUse) throw new Error('wrong token_use');
    if ((tokenUse === 'id' ? claims.aud : claims.client_id) !== cfg.clientId) throw new Error('wrong client');
    if (typeof claims.exp !== 'number' || claims.exp <= t + EXPIRY_MARGIN_S) throw new Error('expired');
    if (typeof claims.iat === 'number' && claims.iat > t + CLOCK_SKEW_S) throw new Error('issued in the future');
    if (typeof claims.sub !== 'string' || !claims.sub) throw new Error('no subject');
    return claims;
  }

  /** The session of the cookies: { sub, exp } or null. */
  async function verifySession(tokens, cfg) {
    try {
      const id = await verifyToken(tokens.id, 'id', cfg);
      const access = await verifyToken(tokens.access, 'access', cfg);
      if (id.sub !== access.sub) throw new Error('tokens of different users');
      return { sub: id.sub, exp: Math.min(id.exp, access.exp) };
    } catch {
      return null;
    }
  }

  async function tokenRequest(cfg, form) {
    const r = await fetchWithTimeout(`https://${cfg.cognitoDomain}/oauth2/token`, {
      method: 'POST',
      headers: { 'content-type': 'application/x-www-form-urlencoded' },
      body: new URLSearchParams({ client_id: cfg.clientId, ...form }).toString(),
    });
    if (!r.ok) throw new Error(`token endpoint ${r.status}`);
    return r.json();
  }

  function sessionCookies(cfg, tokens, session) {
    const t = now() / 1000;
    const cookies = [
      setCookie(COOKIES.id, tokens.id, session.exp - t),
      setCookie(COOKIES.access, tokens.access, session.exp - t),
    ];
    if (tokens.newRefresh) cookies.push(setCookie(COOKIES.refresh, tokens.refresh, cfg.refreshTtlSeconds));
    return cookies;
  }

  async function refresh(cfg, refreshToken) {
    try {
      const r = await tokenRequest(cfg, { grant_type: 'refresh_token', refresh_token: refreshToken });
      const tokens = { id: r.id_token, access: r.access_token, refresh: r.refresh_token || refreshToken, newRefresh: Boolean(r.refresh_token) };
      const session = await verifySession(tokens, cfg);
      return session ? { tokens, session } : null;
    } catch (e) {
      log({ gate: 'refresh-failed', error: String(e?.message || e) });
      return null;
    }
  }

  function callbackUrl(cfg) {
    return `https://${cfg.distributionDomain}${CALLBACK_PATH}`;
  }

  function loginRedirect(cfg, returnPath, extraCookies = []) {
    const state = randomToken(random);
    const nonce = randomToken(random);
    const verifier = randomToken(random, 48);
    const challenge = crypto.createHash('sha256').update(verifier).digest('base64url');
    const loginCookie = Buffer.from(JSON.stringify({ s: state, n: nonce, v: verifier, r: safeReturnPath(returnPath) })).toString('base64url');
    const q = new URLSearchParams({
      response_type: 'code',
      client_id: cfg.clientId,
      redirect_uri: callbackUrl(cfg),
      scope: 'openid email profile',
      state,
      nonce,
      code_challenge: challenge,
      code_challenge_method: 'S256',
    });
    return redirect(302, `https://${cfg.cognitoDomain}/oauth2/authorize?${q}`, [
      ...extraCookies,
      setCookie(COOKIES.login, loginCookie, LOGIN_COOKIE_TTL_S),
    ]);
  }

  async function handleCallback(request, cfg, cookies) {
    const q = new URLSearchParams(request.querystring || '');
    const clearLogin = setCookie(COOKIES.login, '', 0);
    if (q.get('error')) {
      return respond(400, `Sign-in failed (${q.get('error').slice(0, 100)}). Open / to try again.\n`, { cookies: [clearLogin] });
    }
    let login;
    try {
      login = b64urlJson(cookies[COOKIES.login]);
    } catch {
      return respond(400, 'Sign-in expired or was started elsewhere. Open / to try again.\n', { cookies: [clearLogin] });
    }
    const code = q.get('code');
    if (!code || !/^[A-Za-z0-9._~-]{1,512}$/.test(code) || !safeEqual(q.get('state') || '', String(login.s || ''))) {
      return respond(400, 'Sign-in state does not match. Open / to try again.\n', { cookies: [clearLogin] });
    }
    let tokens;
    try {
      const r = await tokenRequest(cfg, {
        grant_type: 'authorization_code', code, redirect_uri: callbackUrl(cfg), code_verifier: String(login.v || ''),
      });
      tokens = { id: r.id_token, access: r.access_token, refresh: r.refresh_token, newRefresh: Boolean(r.refresh_token) };
    } catch (e) {
      log({ gate: 'code-exchange-failed', error: String(e?.message || e) });
      return respond(400, 'Sign-in could not be completed. Open / to try again.\n', { cookies: [clearLogin] });
    }
    const session = await verifySession(tokens, cfg);
    let nonceOk = false;
    try {
      nonceOk = safeEqual(b64urlJson(tokens.id.split('.')[1]).nonce, String(login.n || ''));
    } catch { /* not a token */ }
    if (!session || !nonceOk) {
      return respond(400, 'Sign-in returned an invalid session. Open / to try again.\n', { cookies: [clearLogin] });
    }
    log({ gate: 'login', sub: session.sub });
    return redirect(302, safeReturnPath(login.r), [...sessionCookies(cfg, tokens, session), clearLogin]);
  }

  async function handleLogout(cfg, cookies) {
    const token = cookies[COOKIES.refresh];
    if (token) {
      try {
        const r = await fetchWithTimeout(`https://${cfg.cognitoDomain}/oauth2/revoke`, {
          method: 'POST',
          headers: { 'content-type': 'application/x-www-form-urlencoded' },
          body: new URLSearchParams({ token, client_id: cfg.clientId }).toString(),
        });
        if (!r.ok) log({ gate: 'revoke-failed', status: r.status });
      } catch (e) {
        log({ gate: 'revoke-failed', error: String(e?.message || e) });
      }
    }
    const q = new URLSearchParams({ client_id: cfg.clientId, logout_uri: `https://${cfg.distributionDomain}/` });
    return redirect(302, `https://${cfg.cognitoDomain}/logout?${q}`, clearCookies([...SESSION_COOKIES, COOKIES.login]));
  }

  function forward(request, cfg, sub) {
    dropOwnCookies(request);
    request.headers['x-user-alias'] = [{ key: 'X-User-Alias', value: 'default' }];
    request.headers['x-forwarded-host'] = [{ key: 'X-Forwarded-Host', value: cfg.distributionDomain }];
    // Informational (the backend's logs), never an identity: the backend authorizes on X-User-Alias only.
    request.headers['x-audit-sub'] = [{ key: 'X-Audit-Sub', value: sub }];
    // The proof for the backend that the request came through this gate (never logged).
    request.headers[ORIGIN_HEADER] = [{ key: 'X-Origin-Verify', value: cfg.originSecret }];
    log({ gate: 'allow', sub, method: request.method, uri: request.uri });
    return request;
  }

  const gate = async function handler(event) {
    const request = event.Records[0].cf.request;
    request.headers = request.headers || {};
    stripHeaders(request);

    let cfg;
    try {
      cfg = await getConfig();
    } catch (e) {
      log({ gate: 'no-config', error: String(e?.message || e) });
      return respond(503, 'The sign-in gate is not configured yet.\n', { extra: { 'retry-after': [{ key: 'Retry-After', value: '30' }] } });
    }

    const self = `https://${cfg.distributionDomain}`;
    const origin = header(request, 'origin');
    const upgrade = isUpgrade(request);
    if (upgrade && origin !== self) return forbidden('WebSocket from a foreign origin');
    if (!['GET', 'HEAD', 'OPTIONS'].includes(request.method) && origin && origin !== self) {
      return forbidden('request from a foreign origin');
    }

    const cookies = parseCookies(request);
    const uri = request.uri || '/';
    if (uri.startsWith(AUTH_PREFIX)) {
      if (uri === CALLBACK_PATH && request.method === 'GET') return handleCallback(request, cfg, cookies);
      if (uri === LOGOUT_PATH && (request.method === 'GET' || request.method === 'POST')) return handleLogout(cfg, cookies);
      if (uri === LOGIN_PATH && request.method === 'GET') {
        return loginRedirect(cfg, new URLSearchParams(request.querystring || '').get('return') || '/');
      }
      return respond(404, 'Not found\n');
    }

    const tokens = { id: cookies[COOKIES.id], access: cookies[COOKIES.access] };
    const session = tokens.id || tokens.access ? await verifySession(tokens, cfg) : null;
    if (session) return forward(request, cfg, session.sub);

    const here = uri + (request.querystring ? `?${request.querystring}` : '');
    if (cookies[COOKIES.refresh]) {
      const fresh = await refresh(cfg, cookies[COOKIES.refresh]);
      if (fresh) {
        const set = sessionCookies(cfg, fresh.tokens, fresh.session);
        // A WebSocket cannot follow a redirect or store cookies: let this one through; the next HTTP request stores
        // the tokens. With refresh-token rotation (the pool client's RetryGracePeriodSeconds, 60 s) the refresh token
        // the browser still has keeps working for that grace period after this refresh; the viewer's health poll
        // (every 5 s, crates/factoryos-viewer/src/reconnect.rs) refreshes again within it and stores the new tokens.
        if (upgrade) return forward(request, cfg, fresh.session.sub);
        // The browser stores the new tokens and repeats the request (307: with its method and body).
        // safeReturnPath: the raw URI may be //host or /\host, a redirect off this site.
        return redirect(request.method === 'GET' || request.method === 'HEAD' ? 302 : 307, safeReturnPath(here), set);
      }
    }
    const stale = Object.keys(cookies).some((n) => SESSION_COOKIES.includes(n)) ? clearCookies(SESSION_COOKIES) : [];
    log({ gate: 'deny', method: request.method, uri });
    if (isNavigation(request)) return loginRedirect(cfg, here, stale);
    return unauthorized(stale);
  };
  gate.ready = ready;
  return gate;
}

export const handler = createHandler();
// INIT waits for the preload: an ES module's top-level await is part of INIT (its own ~10 s budget, not the 5 s of a
// request). Without it INIT ends at once and the load runs inside the first request after all (test install rf-old3,
// from eu-central-1: INIT 150 ms, then each cold request timed out at 5 s or answered "not configured yet").
await handler.ready;
