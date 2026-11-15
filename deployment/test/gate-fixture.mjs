// A user pool of the tests' own for the edge gate: a key pair served as its JWKS, a token endpoint, CloudFront events.
import crypto from 'node:crypto';
import { createHandler } from '../lambda/edge-auth/index.mjs';

export function gateFixture() {
  const region = 'eu-central-1';
  const cfg = {
    format: 1,
    region,
    userPoolId: `${region}_TestPool`,
    clientId: 'testclient0123456789abcdef',
    cognitoDomain: 'login.example.com',
    distributionDomain: 'factoryos.example.com',
    refreshTtlSeconds: 43200,
    // The origin secret the gate adds as x-origin-verify (in the stack: the installer's SecureString edge-origin-secret).
    originSecret: 'test-origin-secret_0123456789abcdefghijklmnopqrstuvwxyzABCD', // pragma: allowlist secret (the test's own)
  };
  const issuer = `https://cognito-idp.${region}.amazonaws.com/${cfg.userPoolId}`;
  const kid = 'test-key-1';
  const { privateKey, publicKey } = crypto.generateKeyPairSync('rsa', { modulusLength: 2048 });
  const jwks = { keys: [{ ...publicKey.export({ format: 'jwk' }), kid, alg: 'RS256', use: 'sig' }] };
  const sub = '11111111-2222-4333-8444-555555555555';

  const b64 = (o) => Buffer.from(JSON.stringify(o)).toString('base64url');
  const claims = (use, extra = {}) => {
    const t = Math.floor(Date.now() / 1000);
    return {
      sub,
      iss: issuer,
      token_use: use,
      ...(use === 'id' ? { aud: cfg.clientId, email: 'user@example.com' } : { client_id: cfg.clientId, scope: 'openid email profile' }),
      iat: t,
      exp: t + 3600,
      ...extra,
    };
  };
  const token = (c, { key = privateKey, kid: k = kid, alg = 'RS256' } = {}) => {
    const data = `${b64({ alg, kid: k })}.${b64(c)}`;
    return `${data}.${crypto.sign('RSA-SHA256', Buffer.from(data), key).toString('base64url')}`;
  };
  const pair = (extra = {}) => {
    const { aud, client_id: clientId, ...common } = extra;
    return {
      id: token(claims('id', { ...common, ...(aud ? { aud } : {}) })),
      access: token(claims('access', { ...common, ...(clientId ? { client_id: clientId } : {}) })),
    };
  };

  /** A handler with fakes: { handler, calls, nonce } (set `nonce` for the ID token the token endpoint issues). */
  function gate({ loadConfig, tokenStatus = 200, issueTokens } = {}) {
    const g = { calls: [], nonce: undefined };
    const fetchImpl = async (url, init = {}) => {
      g.calls.push({ url: String(url), body: init.body });
      const json = (status, body) => ({ ok: status < 300, status, json: async () => body });
      if (String(url) === `${issuer}/.well-known/jwks.json`) return json(200, jwks);
      if (String(url) === `https://${cfg.cognitoDomain}/oauth2/token`) {
        if (tokenStatus !== 200) return json(tokenStatus, { error: 'invalid_grant' });
        if (issueTokens) return json(200, issueTokens());
        const form = new URLSearchParams(init.body);
        const extra = g.nonce ? { nonce: g.nonce } : {};
        const out = { id_token: token(claims('id', extra)), access_token: token(claims('access')), token_type: 'Bearer', expires_in: 3600 };
        if (form.get('grant_type') === 'authorization_code') out.refresh_token = 'new-refresh-token';
        return json(200, out);
      }
      if (String(url) === `https://${cfg.cognitoDomain}/oauth2/revoke`) return json(200, {});
      return json(404, {});
    };
    g.handler = createHandler({
      loadConfig: loadConfig || (async () => JSON.stringify(cfg)),
      fetchImpl,
      log: () => {},
    });
    return g;
  }

  /** A CloudFront viewer-request event (header names lower case, as CloudFront gives them). */
  function event({ method = 'GET', uri = '/', querystring = '', headers = {} } = {}) {
    const h = { host: [{ key: 'Host', value: cfg.distributionDomain }] };
    for (const [k, v] of Object.entries(headers)) {
      if (v === undefined) continue;
      (h[k.toLowerCase()] ||= []).push({ key: k, value: v });
    }
    return { Records: [{ cf: { config: { eventType: 'viewer-request' }, request: { method, uri, querystring, headers: h, clientIp: '192.0.2.10' } } }] };
  }

  const ws = (origin) => ({ 'sec-websocket-key': crypto.randomBytes(16).toString('base64'), 'sec-websocket-version': '13', origin });
  const cookieHeader = (t) => [t.id && `__Host-fos_id=${t.id}`, t.access && `__Host-fos_access=${t.access}`].filter(Boolean).join('; ');
  /** The Set-Cookie of a response for `name`: { value, attrs } or undefined. */
  const cookie = (r, name) => {
    for (const { value } of r.headers?.['set-cookie'] || []) {
      if (value.startsWith(`${name}=`)) {
        const [first, ...rest] = value.split('; ');
        return { value: first.slice(name.length + 1), attrs: rest.join('; ') };
      }
    }
    return undefined;
  };

  return { cfg, issuer, kid, publicKey, sub, b64, claims, token, pair, gate, event, ws, cookieHeader, cookie };
}
