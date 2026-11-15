// FactoryOS public deployment: the VerifyAnonymousDenial custom resource (CloudFormation calls this Lambda directly).
//
// Before the stack opens the origin (the instance's port-80 ingress from CloudFront's VPC origins), it checks as an
// anonymous client that the distribution's gate denies everything (the denial matrix):
//   - a page load of / goes to the pool's /oauth2/authorize with this client (302);
//   - /api/*, HEAD, POST and WebSocket upgrades get 401 with the gate's WWW-Authenticate (realm="factoryos");
//   - a WebSocket upgrade from a foreign Origin is refused (401/403);
//   - a request carrying its own x-origin-verify (the header the gate adds with the origin secret) is denied the same
//     way: knowing or guessing the secret gets no anonymous request through CloudFront;
//   - garbage, alg=none, expired and wrong-pool tokens in the session cookies are denied the same way;
//   - the login's callback without its state is refused (4xx);
//   - nothing ever answers 2xx (or 101, an upgraded WebSocket): that fails the deployment at once.
// Anything else (CloudFront still propagating, the old edge version, 5xx) is retried until the deadline (~12 min).
// Node standard library only.

import crypto from 'node:crypto';
import https from 'node:https';

// The gate's session cookies (lambda/edge-auth/index.mjs COOKIES; a unit test keeps the two in step).
export const ID_COOKIE = '__Host-fos_id';
export const ACCESS_COOKIE = '__Host-fos_access';
const REALM = 'realm="factoryos"';
const MAX_WAIT_MS = 12 * 60 * 1000;
const RETRY_MS = 20 * 1000;

const b64 = (o) => Buffer.from(JSON.stringify(o)).toString('base64url');

/** Tokens no gate may accept: unsigned, expired, of another pool, garbage. */
export function forgedTokens({ issuer, clientId, region, now = Date.now() }) {
  const t = Math.floor(now / 1000);
  const { privateKey } = crypto.generateKeyPairSync('rsa', { modulusLength: 2048 });
  const sign = (head, claims) => {
    const data = `${b64(head)}.${b64(claims)}`;
    return `${data}.${crypto.sign('RSA-SHA256', Buffer.from(data), privateKey).toString('base64url')}`;
  };
  const claims = (use, extra = {}) => ({
    sub: '00000000-0000-4000-8000-000000000000',
    iss: issuer,
    token_use: use,
    ...(use === 'id' ? { aud: clientId } : { client_id: clientId }),
    iat: t - 60,
    exp: t + 3600,
    ...extra,
  });
  const pair = (fn) => ({ id: fn('id'), access: fn('access') });
  const otherPool = `https://cognito-idp.${region}.amazonaws.com/${region}_verifyDenialOtherPool`;
  return {
    garbage: { id: 'not-a-token', access: `${b64({ alg: 'RS256' })}.${b64({})}.AAAA` },
    'alg-none': pair((use) => `${b64({ alg: 'none', typ: 'JWT' })}.${b64(claims(use))}.`),
    expired: pair((use) => sign({ alg: 'RS256', kid: 'verify-denial' }, claims(use, { iat: t - 7200, exp: t - 3600 }))),
    'wrong-pool': pair((use) => sign({ alg: 'RS256', kid: 'verify-denial' }, claims(use, { iss: otherPool }))),
  };
}

/** One HTTPS request, no redirects followed: { status, headers } (101 for an upgraded WebSocket). */
export function httpsRequest(base) {
  return ({ method, path, headers }) => new Promise((resolve, reject) => {
    const req = https.request(new URL(path, base), { method, headers, timeout: 15000 });
    req.on('response', (res) => {
      res.resume();
      resolve({ status: res.statusCode, headers: res.headers });
    });
    req.on('upgrade', (res, socket) => {
      socket.destroy();
      resolve({ status: 101, headers: res.headers });
    });
    req.on('timeout', () => req.destroy(new Error('timeout')));
    req.on('error', reject);
    req.end();
  });
}

function hdr(headers, name) {
  const v = headers?.[name];
  return Array.isArray(v) ? v.join(', ') : v || '';
}

/** The checks of the matrix: [{ name, request, expect(response) -> problem string | null }]. */
export function matrix({ base, clientId, cognitoDomain, issuer, region, now }) {
  const page = { accept: 'text/html,application/xhtml+xml', 'sec-fetch-mode': 'navigate', 'sec-fetch-dest': 'document' };
  const ws = (origin) => ({
    connection: 'Upgrade',
    upgrade: 'websocket',
    'sec-websocket-version': '13',
    'sec-websocket-key': crypto.randomBytes(16).toString('base64'),
    origin,
  });
  const toLogin = (r) => {
    if (r.status !== 302) return `expected 302 to the login, got ${r.status}`;
    let u;
    try {
      u = new URL(hdr(r.headers, 'location'));
    } catch {
      return 'redirect without a valid Location';
    }
    if (u.origin !== `https://${cognitoDomain}` || u.pathname !== '/oauth2/authorize') return `redirect to ${u.origin}${u.pathname}`;
    if (u.searchParams.get('client_id') !== clientId) return 'redirect with another client_id';
    if (u.searchParams.get('redirect_uri') !== `${base}/_auth/callback`) return 'redirect with another redirect_uri';
    if (u.searchParams.get('code_challenge_method') !== 'S256' || !u.searchParams.get('state')) return 'redirect without PKCE/state';
    return null;
  };
  const gate401 = (r) => {
    if (r.status !== 401) return `expected 401, got ${r.status}`;
    return hdr(r.headers, 'www-authenticate').includes(REALM) ? null : 'a 401 without the gate\'s WWW-Authenticate';
  };
  const refused = (r) => (r.status === 401 || r.status === 403 ? null : `expected 401/403, got ${r.status}`);
  const clientError = (r) => (r.status >= 400 && r.status < 500 ? null : `expected 4xx, got ${r.status}`);

  const checks = [
    { name: 'page load of /', request: { method: 'GET', path: '/', headers: page }, expect: toLogin },
    { name: 'API GET', request: { method: 'GET', path: '/api/v1/health', headers: { accept: 'application/json' } }, expect: gate401 },
    { name: 'API GET with a client-sent x-origin-verify', request: { method: 'GET', path: '/api/v1/health',
      headers: { accept: 'application/json', 'x-origin-verify': crypto.randomBytes(48).toString('base64url') } }, expect: gate401 },
    { name: 'HEAD /', request: { method: 'HEAD', path: '/', headers: {} }, expect: gate401 },
    { name: 'API POST', request: { method: 'POST', path: '/api/v1/health', headers: { origin: base, 'content-length': '0' } }, expect: gate401 },
    { name: 'WebSocket /zenoh', request: { method: 'GET', path: '/zenoh', headers: ws(base) }, expect: gate401 },
    { name: 'WebSocket /chat', request: { method: 'GET', path: '/chat', headers: ws(base) }, expect: gate401 },
    { name: 'WebSocket, foreign Origin', request: { method: 'GET', path: '/chat', headers: ws('https://attacker.example') }, expect: refused },
    { name: 'callback without state', request: { method: 'GET', path: '/_auth/callback?code=x&state=y', headers: page }, expect: clientError },
  ];
  for (const [kind, t] of Object.entries(forgedTokens({ issuer, clientId, region, now }))) {
    const cookie = `${ID_COOKIE}=${t.id}; ${ACCESS_COOKIE}=${t.access}`;
    checks.push({ name: `${kind} token, API`, request: { method: 'GET', path: '/api/v1/health', headers: { cookie } }, expect: gate401 });
    checks.push({ name: `${kind} token, page load`, request: { method: 'GET', path: '/', headers: { ...page, cookie } }, expect: toLogin });
  }
  return checks;
}

/** One pass over the matrix: { passed, fatal, problems }. A 2xx or 101 anywhere is fatal. */
export async function runMatrix(props, request) {
  const problems = [];
  let fatal = null;
  for (const c of matrix(props)) {
    let r;
    try {
      r = await request(c.request);
    } catch (e) {
      problems.push(`${c.name}: ${e.message}`);
      continue;
    }
    if ((r.status >= 200 && r.status < 300) || r.status === 101) {
      fatal = `${c.name}: answered ${r.status} without a session`;
      break;
    }
    const p = c.expect(r);
    if (p) problems.push(`${c.name}: ${p}`);
  }
  return { passed: !fatal && problems.length === 0, fatal, problems };
}

/** Repeats the matrix until it passes, a pass is fatal, or the deadline. */
export async function verify(props, { request, deadline, sleep = (ms) => new Promise((r) => setTimeout(r, ms)), log = console.log }) {
  for (let round = 1; ; round++) {
    const r = await runMatrix(props, request);
    log(JSON.stringify({ round, passed: r.passed, fatal: r.fatal, problems: r.problems }));
    if (r.fatal) throw new Error(`the gate let an anonymous request through: ${r.fatal}`);
    if (r.passed) return round;
    if (Date.now() + RETRY_MS > deadline) {
      throw new Error(`the denial matrix did not pass in time: ${r.problems.slice(0, 5).join('; ')}`);
    }
    await sleep(RETRY_MS);
  }
}

export function propsOf(event) {
  const p = event.ResourceProperties || {};
  const domain = String(p.DistributionDomain || '');
  if (!/^[a-z0-9.-]+$/.test(domain)) throw new Error('DistributionDomain is missing');
  return {
    base: `https://${domain}`,
    clientId: String(p.ClientId || ''),
    cognitoDomain: String(p.CognitoDomain || ''),
    issuer: String(p.Issuer || ''),
    region: String(p.Region || ''),
  };
}

function sendResponse(event, body) {
  const data = JSON.stringify(body);
  return new Promise((resolve, reject) => {
    const req = https.request(event.ResponseURL, {
      method: 'PUT',
      headers: { 'content-type': '', 'content-length': Buffer.byteLength(data) },
    }, (res) => {
      res.resume();
      res.on('end', resolve);
    });
    req.on('error', reject);
    req.end(data);
  });
}

/** The custom resource's work; `send` answers CloudFormation, `requestFor(base)` makes the checks' requests. */
export async function handle(event, context, { send = sendResponse, requestFor = httpsRequest, sleep } = {}) {
  const physicalId = event.PhysicalResourceId || `verify-anonymous-denial-${event.LogicalResourceId}`;
  const base = { StackId: event.StackId, RequestId: event.RequestId, LogicalResourceId: event.LogicalResourceId, PhysicalResourceId: physicalId };
  if (event.RequestType === 'Delete') {
    await send(event, { ...base, Status: 'SUCCESS' });
    return;
  }
  let result;
  try {
    const props = propsOf(event);
    const deadline = Date.now() + Math.min(MAX_WAIT_MS, context.getRemainingTimeInMillis() - 60 * 1000);
    const rounds = await verify(props, { request: requestFor(props.base), deadline, ...(sleep ? { sleep } : {}) });
    result = { ...base, Status: 'SUCCESS', Data: { Verified: 'true', Rounds: String(rounds) } };
  } catch (e) {
    console.log(JSON.stringify({ error: String(e?.message || e) }));
    result = { ...base, Status: 'FAILED', Reason: String(e?.message || e).slice(0, 1000) };
  }
  await send(event, result);
}

export const handler = (event, context) => handle(event, context);
