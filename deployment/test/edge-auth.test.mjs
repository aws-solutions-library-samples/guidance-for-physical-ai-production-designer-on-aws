// The Lambda@Edge gate (lambda/edge-auth/index.mjs): the denial matrix, header hygiene, token checks, cookies, the
// login / refresh / logout flows. Tokens are minted with a key pair of the test's own, served as the pool's JWKS.
import crypto from 'node:crypto';
import { readFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';
import {
  COOKIES, ORIGIN_HEADER, STRIPPED_HEADERS, createHandler, parseConfig, safeReturnPath, secretParameterName, setCookie,
  withOriginSecret,
} from '../lambda/edge-auth/index.mjs';
import { gateFixture } from './gate-fixture.mjs';

const f = gateFixture();
const { cfg } = f;

describe('without a session', () => {
  it('sends a page load of / to the managed login with PKCE, state and nonce', async () => {
    const { handler } = f.gate();
    const r = await handler(f.event({ uri: '/', headers: { accept: 'text/html', 'sec-fetch-mode': 'navigate' } }));
    expect(r.status).toBe('302');
    const loc = new URL(r.headers.location[0].value);
    expect(`${loc.origin}${loc.pathname}`).toBe(`https://${cfg.cognitoDomain}/oauth2/authorize`);
    const q = loc.searchParams;
    expect(q.get('client_id')).toBe(cfg.clientId);
    expect(q.get('response_type')).toBe('code');
    expect(q.get('redirect_uri')).toBe(`https://${cfg.distributionDomain}/_auth/callback`);
    expect(q.get('scope')).toBe('openid email profile');
    expect(q.get('code_challenge_method')).toBe('S256');
    expect(q.get('code_challenge')).toMatch(/^[A-Za-z0-9_-]{43}$/);
    expect(q.get('state')).toBeTruthy();
    expect(q.get('nonce')).toBeTruthy();
    const login = f.cookie(r, COOKIES.login);
    expect(login).toBeTruthy();
    const saved = JSON.parse(Buffer.from(login.value, 'base64url').toString());
    expect(saved.s).toBe(q.get('state'));
    expect(saved.n).toBe(q.get('nonce'));
    expect(crypto.createHash('sha256').update(saved.v).digest('base64url')).toBe(q.get('code_challenge'));
  });

  it('a GET without fetch metadata and with Accept text/html is a page load too', async () => {
    const r = await f.gate().handler(f.event({ uri: '/cells', headers: { accept: 'text/html' } }));
    expect(r.status).toBe('302');
  });

  const denied = [
    ['API GET', { uri: '/api/v1/health', headers: { accept: 'application/json' } }],
    ['API GET that looks like a page load', { uri: '/api/v1/health', headers: { accept: 'text/html' } }],
    ['fetch() of a page path', { uri: '/', headers: { 'sec-fetch-mode': 'cors' } }],
    ['HEAD /', { method: 'HEAD', uri: '/' }],
    ['POST', { method: 'POST', uri: '/api/v1/cells', headers: { origin: `https://${cfg.distributionDomain}` } }],
    ['DELETE', { method: 'DELETE', uri: '/api/v1/cells/x' }],
    ['OPTIONS', { method: 'OPTIONS', uri: '/api/v1/cells' }],
    ['WebSocket /zenoh', { uri: '/zenoh', headers: f.ws(`https://${cfg.distributionDomain}`) }],
    ['WebSocket /chat', { uri: '/chat', headers: f.ws(`https://${cfg.distributionDomain}`) }],
  ];
  for (const [name, req] of denied) {
    it(`answers ${name} with 401 and the gate's WWW-Authenticate`, async () => {
      const r = await f.gate().handler(f.event(req));
      expect(r.status).toBe('401');
      expect(r.headers['www-authenticate'][0].value).toContain('realm="factoryos"');
      expect(r.headers['cache-control'][0].value).toBe('no-store');
    });
  }

  it('refuses a WebSocket from a foreign Origin (or none)', async () => {
    for (const origin of ['https://attacker.example', undefined, `http://${cfg.distributionDomain}`]) {
      const r = await f.gate().handler(f.event({ uri: '/chat', headers: f.ws(origin) }));
      expect(r.status).toBe('403');
    }
  });

  it('answers 503 (never passes) without its configuration', async () => {
    const { handler } = f.gate({ loadConfig: async () => { throw new Error('ParameterNotFound'); } });
    const r = await handler(f.event({ uri: '/api/v1/health' }));
    expect(r.status).toBe('503');
    const bad = f.gate({ loadConfig: async () => JSON.stringify({ ...cfg, clientId: '' }) });
    expect((await bad.handler(f.event({ uri: '/' }))).status).toBe('503');
  });

  it('answers unknown /_auth/ paths with 404', async () => {
    expect((await f.gate().handler(f.event({ uri: '/_auth/whatever' }))).status).toBe('404');
  });
});

describe('token checks', () => {
  const t = () => Math.floor(Date.now() / 1000);
  const other = crypto.generateKeyPairSync('rsa', { modulusLength: 2048 });
  const cases = {
    garbage: () => ({ id: 'garbage', access: 'garbage' }),
    'three dots of garbage': () => ({ id: 'a.b.c', access: '!!.??.##' }),
    expired: () => f.pair({ exp: t() - 10 }),
    'expiring within the margin': () => f.pair({ exp: t() + 5 }),
    'another pool (correct key)': () => f.pair({ iss: `https://cognito-idp.${cfg.region}.amazonaws.com/${cfg.region}_OtherPool1` }),
    'another client': () => f.pair({ aud: 'otherclient0000000000000', client_id: 'otherclient0000000000000' }),
    'alg=none': () => {
      const none = (use) => `${f.b64({ alg: 'none', kid: f.kid })}.${f.b64(f.claims(use))}.`;
      return { id: none('id'), access: none('access') };
    },
    'alg=HS256 with the public key as secret': () => {
      const pem = f.publicKey.export({ type: 'spki', format: 'pem' });
      const hs = (use) => {
        const data = `${f.b64({ alg: 'HS256', kid: f.kid })}.${f.b64(f.claims(use))}`;
        return `${data}.${crypto.createHmac('sha256', pem).update(data).digest('base64url')}`;
      };
      return { id: hs('id'), access: hs('access') };
    },
    'signed by another key with the pool key id': () => ({
      id: f.token(f.claims('id'), { key: other.privateKey }),
      access: f.token(f.claims('access'), { key: other.privateKey }),
    }),
    'unknown key id': () => ({ id: f.token(f.claims('id'), { kid: 'nope' }), access: f.token(f.claims('access'), { kid: 'nope' }) }),
    'wrong token_use (swapped)': () => {
      const p = f.pair();
      return { id: p.access, access: p.id };
    },
    'access token as both': () => {
      const p = f.pair();
      return { id: p.access, access: p.access };
    },
    'tokens of two users': () => ({ id: f.token(f.claims('id', { sub: 'a' })), access: f.token(f.claims('access', { sub: 'b' })) }),
    'only an ID token': () => ({ id: f.pair().id }),
    'tampered claims': () => {
      const p = f.pair();
      const [h, , s] = p.id.split('.');
      return { id: `${h}.${f.b64(f.claims('id', { sub: 'someone-else' }))}.${s}`, access: p.access };
    },
  };
  for (const [name, make] of Object.entries(cases)) {
    it(`denies ${name}`, async () => {
      const tokens = make();
      const api = await f.gate().handler(f.event({ uri: '/api/v1/health', headers: { cookie: f.cookieHeader(tokens) } }));
      expect(api.status).toBe('401');
      // The bad cookies are cleared, so the browser stops sending them.
      expect(f.cookie(api, COOKIES.id)?.attrs).toContain('Max-Age=0');
      const page = await f.gate().handler(f.event({ uri: '/', headers: { accept: 'text/html', cookie: f.cookieHeader(tokens) } }));
      expect(page.status).toBe('302');
      expect(page.headers.location[0].value).toContain('/oauth2/authorize?');
    });
  }

  it('accepts a valid pair', async () => {
    const r = await f.gate().handler(f.event({ uri: '/api/v1/health', headers: { cookie: f.cookieHeader(f.pair()) } }));
    expect(r.status).toBeUndefined();
    expect(r.uri).toBe('/api/v1/health');
  });
});

describe('with a session', () => {
  it('passes the request on with only the gate\'s identity headers', async () => {
    const forged = {
      'x-user-alias': 'admin',
      'x-workspace': 'other',
      'x-terminal-principal': 'arn:aws:iam::111122223333:role/x',
      'x-isaac-principal': 'x',
      'x-smartpad-site': 'evil.example',
      'x-forwarded-host': 'evil.example',
      'x-origin-verify': 'guess',
      'x-amzn-oidc-data': 'x',
      'x-amzn-trace-id': 'x',
      'x-audit-sub': 'someone',
    };
    const cookie = `${f.cookieHeader(f.pair())}; fos_ws=default; ${COOKIES.refresh}=r`;
    const r = await f.gate().handler(f.event({ uri: '/api/v1/cells', headers: { ...forged, cookie, 'X-User-Alias': 'admin2' } }));
    expect(r.uri).toBe('/api/v1/cells');
    expect(r.headers['x-user-alias']).toEqual([{ key: 'X-User-Alias', value: 'default' }]);
    expect(r.headers['x-forwarded-host']).toEqual([{ key: 'X-Forwarded-Host', value: cfg.distributionDomain }]);
    expect(r.headers['x-audit-sub'][0].value).toBe(f.sub);
    // The client's x-origin-verify ('guess') is replaced by the gate's own.
    expect(r.headers[ORIGIN_HEADER]).toEqual([{ key: 'X-Origin-Verify', value: cfg.originSecret }]);
    for (const name of [...STRIPPED_HEADERS, 'x-amzn-oidc-data', 'x-amzn-trace-id', 'X-User-Alias']) {
      if (['x-user-alias', 'x-forwarded-host', 'x-audit-sub', ORIGIN_HEADER].includes(name)) continue;
      expect(r.headers[name], name).toBeUndefined();
    }
    // The tokens never reach the backend; its own cookies do.
    expect(r.headers.cookie[0].value).toBe('fos_ws=default');
  });

  it('strips the identity headers before anything else (also on denied requests)', async () => {
    const req = f.event({ uri: '/api/v1/health', headers: { 'x-user-alias': 'admin' } });
    await f.gate().handler(req);
    expect(req.Records[0].cf.request.headers['x-user-alias']).toBeUndefined();
  });

  it('lets a same-origin WebSocket through and refuses a foreign one', async () => {
    const cookie = f.cookieHeader(f.pair());
    const ok = await f.gate().handler(f.event({ uri: '/zenoh', headers: { ...f.ws(`https://${cfg.distributionDomain}`), cookie } }));
    expect(ok.uri).toBe('/zenoh');
    const foreign = await f.gate().handler(f.event({ uri: '/zenoh', headers: { ...f.ws('https://attacker.example'), cookie } }));
    expect(foreign.status).toBe('403');
  });

  it('refuses a state-changing request from a foreign Origin', async () => {
    const r = await f.gate().handler(f.event({
      method: 'POST', uri: '/api/v1/cells', headers: { origin: 'https://attacker.example', cookie: f.cookieHeader(f.pair()) },
    }));
    expect(r.status).toBe('403');
  });
});

describe('the origin secret', () => {
  const secretOf = (r) => r.headers?.[ORIGIN_HEADER];

  it('is added to every forwarded request (API, page, WebSocket, after a refresh) and replaces a client-sent one', async () => {
    const cookie = f.cookieHeader(f.pair());
    const self = `https://${cfg.distributionDomain}`;
    const forwarded = [
      await f.gate().handler(f.event({ uri: '/api/v1/health', headers: { cookie } })),
      await f.gate().handler(f.event({ uri: '/', headers: { cookie, accept: 'text/html', 'X-Origin-Verify': 'forged' } })),
      await f.gate().handler(f.event({ uri: '/zenoh', headers: { ...f.ws(self), cookie, 'x-origin-verify': cfg.originSecret.slice(1) } })),
      await f.gate().handler(f.event({ method: 'POST', uri: '/api/v1/cells', headers: { cookie, origin: self } })),
      await f.gate().handler(f.event({
        uri: '/chat',
        headers: { ...f.ws(self), cookie: `${f.cookieHeader(f.pair({ exp: Math.floor(Date.now() / 1000) - 60 }))}; ${COOKIES.refresh}=rt` },
      })),
    ];
    for (const r of forwarded) {
      expect(r.status, r.uri).toBeUndefined();
      expect(secretOf(r), r.uri).toEqual([{ key: 'X-Origin-Verify', value: cfg.originSecret }]);
      // One header of that name, whatever case the client used.
      expect(Object.keys(r.headers).filter((k) => k.toLowerCase() === ORIGIN_HEADER), r.uri).toEqual([ORIGIN_HEADER]);
    }
  });

  it('is never in a response or a log line, and a denied request loses the client-sent one', async () => {
    const lines = [];
    const g = f.gate();
    const handler = createHandler({ loadConfig: async () => JSON.stringify(cfg), fetchImpl: async () => ({ ok: false, status: 500, json: async () => ({}) }), log: (e) => lines.push(JSON.stringify(e)) });
    const denied = f.event({ uri: '/api/v1/health', headers: { 'x-origin-verify': 'forged' } });
    const responses = [
      await handler(denied),
      await handler(f.event({ uri: '/', headers: { accept: 'text/html' } })),
      await g.handler(f.event({ uri: '/_auth/logout' })),
      await handler(f.event({ uri: '/api/v1/health', headers: { cookie: f.cookieHeader(f.pair()) } })),
    ];
    expect(denied.Records[0].cf.request.headers[ORIGIN_HEADER]).toBeUndefined();
    for (const r of responses.slice(0, 3)) expect(JSON.stringify(r)).not.toContain(cfg.originSecret);
    expect(lines.length).toBeGreaterThan(0);
    for (const l of lines) expect(l).not.toContain(cfg.originSecret);
  });

  it('is required: a configuration without one (or a malformed one) answers 503, never forwards without it', async () => {
    const { originSecret, ...without } = cfg;
    expect(originSecret).toBeTruthy();
    for (const c of [without, { ...cfg, originSecret: 'short' }, { ...cfg, originSecret: `${cfg.originSecret} x` }, { ...cfg, originSecret: 42 }]) { // pragma: allowlist secret (invalid test values)
      const r = await f.gate({ loadConfig: async () => JSON.stringify(c) }).handler(f.event({ uri: '/api/v1/health', headers: { cookie: f.cookieHeader(f.pair()) } }));
      expect(r.status).toBe('503');
      expect(r.headers[ORIGIN_HEADER]).toBeUndefined();
    }
    expect(() => parseConfig(JSON.stringify(without))).toThrow('edge configuration is invalid');
  });

  it('comes from the SecureString next to the configuration parameter', () => {
    expect(secretParameterName({ parameterName: '/factoryos/s1/edge-config', parameterRegion: 'us-east-1' })).toBe('/factoryos/s1/edge-origin-secret');
    expect(secretParameterName({ parameterName: '/factoryos/s1/edge-config', secretParameterName: '/x/y' })).toBe('/x/y');
    expect(() => secretParameterName({ parameterName: '/elsewhere' })).toThrow();
    const { originSecret, ...stored } = cfg;
    const merged = JSON.parse(withOriginSecret(JSON.stringify({ ...stored, originSecret: 'old-value' }), ` ${originSecret}\n`));
    expect(merged.originSecret).toBe(originSecret);
    expect(parseConfig(JSON.stringify(merged)).originSecret).toBe(originSecret);
  });

  it('the code reads the secret with decryption and only with GetParameter', () => {
    const src = readFileSync(new URL('../lambda/edge-auth/index.mjs', import.meta.url), 'utf8');
    expect(src).toContain('WithDecryption: true');
    expect(src).not.toContain('GetParametersCommand');
  });
});

describe('refresh at the edge', () => {
  const expired = () => f.pair({ exp: Math.floor(Date.now() / 1000) - 60 });
  const cookie = () => `${f.cookieHeader(expired())}; ${COOKIES.refresh}=the-refresh-token`;

  it('refreshes run-out tokens and repeats a GET with the new cookies (302)', async () => {
    const g = f.gate();
    const r = await g.handler(f.event({ uri: '/api/v1/health', querystring: 'a=1', headers: { cookie: cookie() } }));
    expect(r.status).toBe('302');
    expect(r.headers.location[0].value).toBe('/api/v1/health?a=1');
    const body = new URLSearchParams(g.calls.find((c) => c.url.endsWith('/oauth2/token')).body);
    expect(body.get('grant_type')).toBe('refresh_token');
    expect(body.get('refresh_token')).toBe('the-refresh-token');
    expect(body.get('client_id')).toBe(cfg.clientId);
    for (const name of [COOKIES.id, COOKIES.access]) {
      const c = f.cookie(r, name);
      expect(c.value.split('.')).toHaveLength(3);
      expect(c.attrs).toMatch(/Max-Age=3[0-9]{3}/);
    }
  });

  it('repeats a POST with 307 and lets a WebSocket through', async () => {
    const post = await f.gate().handler(f.event({ method: 'POST', uri: '/api/v1/cells', headers: { cookie: cookie() } }));
    expect(post.status).toBe('307');
    const ws = await f.gate().handler(f.event({ uri: '/chat', headers: { ...f.ws(`https://${cfg.distributionDomain}`), cookie: cookie() } }));
    expect(ws.uri).toBe('/chat');
    expect(ws.headers['x-user-alias'][0].value).toBe('default');
  });

  it('never redirects off this site after a refresh (//host, /\\host)', async () => {
    for (const uri of ['//evil.example/x', '/\\evil.example/x']) {
      const r = await f.gate().handler(f.event({ uri, headers: { cookie: `${COOKIES.refresh}=the-refresh-token`, 'sec-fetch-mode': 'navigate' } }));
      expect(r.status).toBe('302');
      expect(r.headers.location[0].value).toBe('/');
    }
  });

  it('denies (and clears the cookies) when the refresh fails', async () => {
    const g = f.gate({ tokenStatus: 400 });
    const r = await g.handler(f.event({ uri: '/api/v1/health', headers: { cookie: cookie() } }));
    expect(r.status).toBe('401');
    expect(f.cookie(r, COOKIES.refresh).attrs).toContain('Max-Age=0');
  });

  it('stores a refresh token the token endpoint returns (refresh token rotation), keeps the old one otherwise', async () => {
    const rotating = f.gate({ issueTokens: () => ({ id_token: f.pair().id, access_token: f.pair().access, refresh_token: 'rotated-token' }) });
    const r = await rotating.handler(f.event({ uri: '/api/v1/health', headers: { cookie: cookie() } }));
    expect(r.status).toBe('302');
    expect(f.cookie(r, COOKIES.refresh).value).toBe('rotated-token');
    expect(f.cookie(r, COOKIES.refresh).attrs).toContain(`Max-Age=${cfg.refreshTtlSeconds}`);
    const plain = await f.gate().handler(f.event({ uri: '/api/v1/health', headers: { cookie: cookie() } }));
    expect(plain.status).toBe('302');
    // No new refresh token: the browser keeps its cookie (not re-set, not cleared).
    expect(f.cookie(plain, COOKIES.refresh)).toBeUndefined();
  });

  it('denies when the refreshed tokens do not verify', async () => {
    const g = f.gate({ issueTokens: () => ({ id_token: 'x', access_token: 'y' }) });
    const r = await g.handler(f.event({ uri: '/api/v1/health', headers: { cookie: cookie() } }));
    expect(r.status).toBe('401');
  });
});

describe('login callback', () => {
  async function startLogin(g, uri = '/cells?id=1') {
    const r = await g.handler(f.event({ uri: uri.split('?')[0], querystring: uri.split('?')[1] || '', headers: { accept: 'text/html' } }));
    const q = new URL(r.headers.location[0].value).searchParams;
    return { state: q.get('state'), nonce: q.get('nonce'), loginCookie: f.cookie(r, COOKIES.login).value };
  }

  it('exchanges the code with the PKCE verifier and sets the session cookies', async () => {
    const g = f.gate();
    const { state, nonce, loginCookie } = await startLogin(g);
    g.nonce = nonce;
    const r = await g.handler(f.event({
      uri: '/_auth/callback', querystring: `code=abc123&state=${state}`, headers: { cookie: `${COOKIES.login}=${loginCookie}` },
    }));
    expect(r.status).toBe('302');
    expect(r.headers.location[0].value).toBe('/cells?id=1');
    const body = new URLSearchParams(g.calls.find((c) => c.url.endsWith('/oauth2/token')).body);
    expect(body.get('grant_type')).toBe('authorization_code');
    expect(body.get('code')).toBe('abc123');
    expect(body.get('redirect_uri')).toBe(`https://${cfg.distributionDomain}/_auth/callback`);
    const saved = JSON.parse(Buffer.from(loginCookie, 'base64url').toString());
    expect(body.get('code_verifier')).toBe(saved.v);
    for (const name of [COOKIES.id, COOKIES.access, COOKIES.refresh]) {
      const c = f.cookie(r, name);
      expect(c, name).toBeTruthy();
      expect(c.attrs).toContain('Path=/');
      expect(c.attrs).toContain('Secure');
      expect(c.attrs).toContain('HttpOnly');
      expect(c.attrs).toContain('SameSite=Lax');
      expect(c.attrs).not.toMatch(/Domain=/i);
    }
    expect(f.cookie(r, COOKIES.refresh).attrs).toContain(`Max-Age=${cfg.refreshTtlSeconds}`);
    expect(f.cookie(r, COOKIES.login).attrs).toContain('Max-Age=0');
  });

  it('refuses a callback whose state does not match, without the login cookie, or with a wrong nonce', async () => {
    const g = f.gate();
    const { state, nonce, loginCookie } = await startLogin(g);
    const wrongState = await g.handler(f.event({ uri: '/_auth/callback', querystring: 'code=abc&state=other', headers: { cookie: `${COOKIES.login}=${loginCookie}` } }));
    expect(wrongState.status).toBe('400');
    const noCookie = await g.handler(f.event({ uri: '/_auth/callback', querystring: `code=abc&state=${state}` }));
    expect(noCookie.status).toBe('400');
    g.nonce = `${nonce}x`;
    const wrongNonce = await g.handler(f.event({ uri: '/_auth/callback', querystring: `code=abc&state=${state}`, headers: { cookie: `${COOKIES.login}=${loginCookie}` } }));
    expect(wrongNonce.status).toBe('400');
    expect(f.cookie(wrongNonce, COOKIES.id)).toBeUndefined();
    const error = await g.handler(f.event({ uri: '/_auth/callback', querystring: 'error=access_denied' }));
    expect(error.status).toBe('400');
  });

  it('only returns to paths on this site', () => {
    expect(safeReturnPath('/cells?x=1')).toBe('/cells?x=1');
    for (const bad of ['//evil.example/', '/\\evil.example', 'https://evil.example/', '/_auth/logout', '', '/a\nb']) {
      expect(safeReturnPath(bad), bad).toBe('/');
    }
  });
});

describe('logout', () => {
  it('revokes the refresh token, clears the cookies and goes to the pool\'s logout', async () => {
    const g = f.gate();
    const r = await g.handler(f.event({ uri: '/_auth/logout', headers: { cookie: `${f.cookieHeader(f.pair())}; ${COOKIES.refresh}=rt` } }));
    expect(r.status).toBe('302');
    const loc = new URL(r.headers.location[0].value);
    expect(`${loc.origin}${loc.pathname}`).toBe(`https://${cfg.cognitoDomain}/logout`);
    expect(loc.searchParams.get('client_id')).toBe(cfg.clientId);
    expect(loc.searchParams.get('logout_uri')).toBe(`https://${cfg.distributionDomain}/`);
    const revoke = g.calls.find((c) => c.url.endsWith('/oauth2/revoke'));
    expect(new URLSearchParams(revoke.body).get('token')).toBe('rt');
    for (const name of [COOKIES.id, COOKIES.access, COOKIES.refresh]) expect(f.cookie(r, name).attrs).toContain('Max-Age=0');
  });
});

describe('cookies and code', () => {
  it('every cookie is a __Host- cookie with the strict attributes', () => {
    for (const name of Object.values(COOKIES)) expect(name.startsWith('__Host-')).toBe(true);
    expect(setCookie(COOKIES.id, 'v', 60)).toBe(`${COOKIES.id}=v; Path=/; Secure; HttpOnly; SameSite=Lax; Max-Age=60`);
  });

  it('the edge code knows no internal sign-in', () => {
    const src = readFileSync(new URL('../lambda/edge-auth/index.mjs', import.meta.url), 'utf8').toLowerCase();
    expect(src).not.toContain(['mid', 'way'].join(''));
    expect(src).not.toContain('federat');
  });

  it('createHandler without overrides is the exported handler', () => {
    expect(typeof createHandler()).toBe('function');
  });

  // The cold-start fix: a viewer-request function has 5 s, and loading the SDK and the parameter took ~4.7 s of it.
  it('loads the configuration at INIT, not on the first request (preload)', async () => {
    let calls = 0;
    const loadConfig = async () => { calls += 1; return JSON.stringify(cfg); };
    const handler = createHandler({ loadConfig, log: () => {}, preload: true });
    expect(calls).toBe(1);
    expect((await handler(f.event({ uri: '/api/v1/health' }))).status).toBe('401');
    expect(calls).toBe(1);
  });

  it('preload is off unless the Lambda runtime sets AWS_LAMBDA_FUNCTION_NAME (importing is side-effect free)', () => {
    let calls = 0;
    createHandler({ loadConfig: async () => { calls += 1; return JSON.stringify(cfg); }, log: () => {} });
    expect(calls).toBe(0);
    expect(process.env.AWS_LAMBDA_FUNCTION_NAME).toBeUndefined();
  });

  it('a failed preload is loaded again by the first request, which answers 503', async () => {
    let calls = 0;
    const loadConfig = async () => { calls += 1; throw new Error('ParameterNotFound'); };
    const handler = createHandler({ loadConfig, log: () => {}, preload: true });
    expect(calls).toBe(1);
    // INIT settles before the runtime sends the first invocation.
    await new Promise((r) => { setTimeout(r, 0); });
    expect((await handler(f.event({ uri: '/api/v1/health' }))).status).toBe('503');
    expect(calls).toBe(2);
  });

  // INIT must contain the load: the module awaits `handler.ready` (test install rf-old3: without it INIT ended after
  // 150 ms and each cold request in eu-central-1 ran the SDK import and the us-east-1 calls into its 5 s).
  it('ready settles when the preload has loaded the configuration; the module awaits it at the top level', async () => {
    let release;
    const loadConfig = () => new Promise((r) => { release = () => r(JSON.stringify(cfg)); });
    const handler = createHandler({ loadConfig, log: () => {}, preload: true });
    let settled = false;
    handler.ready.then(() => { settled = true; });
    await new Promise((r) => { setTimeout(r, 0); });
    expect(settled).toBe(false);
    release();
    await handler.ready;
    expect(settled).toBe(true);
    const src = readFileSync(new URL('../lambda/edge-auth/index.mjs', import.meta.url), 'utf8');
    expect(src).toMatch(/^export const handler = createHandler\(\);\n(?:\/\/[^\n]*\n)*await handler\.ready;\s*$/m);
  });

  it('ready never rejects (a failed preload must not fail INIT); without a preload it is resolved', async () => {
    const failing = createHandler({ loadConfig: async () => { throw new Error('AccessDenied'); }, log: () => {}, preload: true });
    await expect(failing.ready).resolves.toBeUndefined();
    await expect(createHandler({ loadConfig: async () => JSON.stringify(cfg), log: () => {} }).ready).resolves.toBeUndefined();
  });

  it('concurrent requests share one load', async () => {
    let calls = 0;
    const loadConfig = async () => { calls += 1; return JSON.stringify(cfg); };
    const handler = createHandler({ loadConfig, log: () => {} });
    const rs = await Promise.all([1, 2, 3].map(() => handler(f.event({ uri: '/api/v1/health' }))));
    expect(rs.map((r) => r.status)).toEqual(['401', '401', '401']);
    expect(calls).toBe(1);
  });
});
