// The VerifyAnonymousDenial check (lambda/verify-denial/index.mjs), run against the real edge gate (in process).
import { describe, expect, it } from 'vitest';
import { COOKIES } from '../lambda/edge-auth/index.mjs';
import {
  ACCESS_COOKIE, ID_COOKIE, handle, matrix, propsOf, runMatrix, verify,
} from '../lambda/verify-denial/index.mjs';
import { gateFixture } from './gate-fixture.mjs';

const f = gateFixture();
const props = {
  base: `https://${f.cfg.distributionDomain}`,
  clientId: f.cfg.clientId,
  cognitoDomain: f.cfg.cognitoDomain,
  issuer: f.issuer,
  region: f.cfg.region,
};

/** Requests answered by a viewer-request handler; a request it passes on reaches the "origin", which answers 200. */
function through(edgeHandler) {
  return async ({ method, path, headers }) => {
    const [uri, querystring = ''] = path.split('?');
    const r = await edgeHandler(f.event({ method, uri, querystring, headers }));
    if (!r.status) return { status: 200, headers: {} };
    return {
      status: Number(r.status),
      headers: Object.fromEntries(Object.entries(r.headers).map(([k, v]) => [k, v.map((x) => x.value).join(', ')])),
    };
  };
}

describe('the denial matrix', () => {
  it('passes against the gate', async () => {
    const r = await runMatrix(props, through(f.gate().handler));
    expect(r.problems).toEqual([]);
    expect(r.fatal).toBeNull();
    expect(r.passed).toBe(true);
  });

  it('covers page loads, the API, HEAD, POST, WebSockets, a foreign Origin and forged tokens', () => {
    const names = matrix(props).map((c) => c.name).join('\n');
    for (const want of ['page load of /', 'API GET', 'HEAD /', 'API POST', 'WebSocket /zenoh', 'WebSocket /chat', 'foreign Origin',
      'garbage token', 'alg-none token', 'expired token', 'wrong-pool token']) {
      expect(names).toContain(want);
    }
  });

  it('is fatal at once when anything answers 2xx without a session', async () => {
    const open = async (event) => event.Records[0].cf.request;
    const r = await runMatrix(props, through(open));
    expect(r.fatal).toMatch(/answered 200/);
    expect(r.passed).toBe(false);
    const upgraded = await runMatrix(props, async () => ({ status: 101, headers: {} }));
    expect(upgraded.fatal).toMatch(/answered 101/);
  });

  it('does not pass a bare 403 (not identifiably the gate) or a redirect to another client', async () => {
    const bare = await runMatrix(props, async () => ({ status: 403, headers: {} }));
    expect(bare.passed).toBe(false);
    expect(bare.fatal).toBeNull();
    const other = await runMatrix({ ...props, clientId: 'anotherclient000000000000' }, through(f.gate().handler));
    expect(other.passed).toBe(false);
    expect(other.problems.join('\n')).toContain('another client_id');
  });

  it('retries until the gate is up, fails at the deadline, throws on a pass-through', async () => {
    let n = 0;
    const gate = through(f.gate().handler);
    const flaky = async (req) => (n++ < 20 ? { status: 503, headers: {} } : gate(req));
    const rounds = await verify(props, { request: flaky, deadline: Date.now() + 10 * 60 * 1000, sleep: async () => {}, log: () => {} });
    expect(rounds).toBeGreaterThan(1);
    await expect(verify(props, { request: async () => ({ status: 502, headers: {} }), deadline: Date.now(), sleep: async () => {}, log: () => {} }))
      .rejects.toThrow(/did not pass in time/);
    await expect(verify(props, { request: async () => ({ status: 200, headers: {} }), deadline: Date.now() + 1e6, sleep: async () => {}, log: () => {} }))
      .rejects.toThrow(/let an anonymous request through/);
  });

  it('uses the gate\'s cookie names', () => {
    expect(ID_COOKIE).toBe(COOKIES.id);
    expect(ACCESS_COOKIE).toBe(COOKIES.access);
  });

  it('reads its properties from the custom resource event', () => {
    const p = propsOf({ ResourceProperties: { DistributionDomain: f.cfg.distributionDomain, ClientId: 'c', CognitoDomain: 'd', Issuer: 'i', Region: 'r' } });
    expect(p.base).toBe(props.base);
    expect(() => propsOf({ ResourceProperties: {} })).toThrow(/DistributionDomain/);
  });

  describe('the custom resource', () => {
    const event = (type) => ({
      RequestType: type, ResponseURL: 'https://cfn.example/', StackId: 's', RequestId: 'r', LogicalResourceId: 'VerifyAnonymousDenial',
      ResourceProperties: { DistributionDomain: f.cfg.distributionDomain, ClientId: f.cfg.clientId, CognitoDomain: f.cfg.cognitoDomain, Issuer: f.issuer, Region: f.cfg.region },
    });
    const context = { getRemainingTimeInMillis: () => 14 * 60 * 1000 };
    const run = async (type, edge) => {
      const sent = [];
      let requests = 0;
      await handle(event(type), context, {
        send: async (_e, body) => { sent.push(body); },
        requestFor: () => { const r = through(edge); return (req) => { requests++; return r(req); }; },
        sleep: async () => {},
      });
      return { sent, requests };
    };

    it('reports Verified=true when the gate denies everything', async () => {
      const { sent } = await run('Create', f.gate().handler);
      expect(sent).toHaveLength(1);
      expect(sent[0].Status).toBe('SUCCESS');
      expect(sent[0].Data.Verified).toBe('true');
      expect(sent[0].PhysicalResourceId).toBe('verify-anonymous-denial-VerifyAnonymousDenial');
    });

    it('fails the deployment when the origin is reachable anonymously', async () => {
      const { sent } = await run('Update', async (e) => e.Records[0].cf.request);
      expect(sent[0].Status).toBe('FAILED');
      expect(sent[0].Reason).toMatch(/anonymous request through/);
    });

    it('answers a Delete without a request', async () => {
      const { sent, requests } = await run('Delete', async (e) => e.Records[0].cf.request);
      expect(sent[0].Status).toBe('SUCCESS');
      expect(requests).toBe(0);
    });
  });
});
