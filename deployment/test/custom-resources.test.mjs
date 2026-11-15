// The stack's own custom resource Lambdas (lambda/custom-resources/): OriginSecret and VpcOriginSecurityGroupLookup,
// with SSM / EC2 and the CloudFormation response faked; and the denial matrix's client-sent x-origin-verify check.
import { describe, expect, it } from 'vitest';
import { EDGE_PARAMETER, handle as originSecret, newSecret, propsOf, SECRET_BYTES, SECRET_PARAMETER } from '../lambda/custom-resources/origin-secret.mjs';
import { handle as sgLookup, lookup } from '../lambda/custom-resources/vpc-origin-sg.mjs';
import { matrix, runMatrix } from '../lambda/verify-denial/index.mjs';
import { gateFixture } from './gate-fixture.mjs';

const PROPS = {
  ServiceToken: 'arn:aws:lambda:eu-central-1:111122223333:function:x',
  SecretParameter: '/factoryos/factoryos/origin-secret',
  SecretRegion: 'eu-central-1', // pragma: allowlist secret (a Region name)
  EdgeParameter: '/factoryos/factoryos/edge-origin-secret',
  EdgeRegion: 'us-east-1',
};
const event = (RequestType, extra = {}) => ({
  RequestType, RequestId: 'req-1', StackId: 'arn:aws:cloudformation:eu-central-1:111122223333:stack/factoryos/x',
  LogicalResourceId: 'OriginSecret', ResponseURL: 'https://example.invalid/', ResourceProperties: PROPS, ...extra,
});

/** A fake SSM per Region: parameters, a call log, optional failures by `${region} ${op}`. */
function fakeSsm({ existing = {}, fail = {} } = {}) {
  const params = { ...existing };
  const calls = [];
  const ssm = async (region) => ({
    put: async (input) => {
      calls.push({ region, op: 'put', input });
      if (fail[`${region} put`]) throw Object.assign(new Error('boom'), { name: fail[`${region} put`] });
      const key = `${region} ${input.Name}`;
      if (params[key] !== undefined && !input.Overwrite) throw Object.assign(new Error('exists'), { name: 'ParameterAlreadyExists' });
      params[key] = input.Value;
      return {};
    },
    del: async (input) => {
      calls.push({ region, op: 'del', input });
      if (fail[`${region} del`]) throw Object.assign(new Error('boom'), { name: fail[`${region} del`] });
      const key = `${region} ${input.Name}`;
      if (params[key] === undefined) throw Object.assign(new Error('not found'), { name: 'ParameterNotFound' });
      delete params[key];
      return {};
    },
  });
  return { ssm, params, calls };
}

async function run(handler, ev, deps) {
  const sent = [];
  const logs = [];
  await handler(ev, {}, { send: async (_e, body) => { sent.push(body); }, log: (l) => logs.push(l), ...deps });
  expect(sent).toHaveLength(1);
  return { response: sent[0], logs };
}

describe('OriginSecret', () => {
  it('a secret is 48 random bytes, base64url (64 characters), new every time', () => {
    const a = newSecret();
    expect(SECRET_BYTES).toBe(48);
    expect(a).toMatch(/^[A-Za-z0-9_-]{64}$/);
    expect(Buffer.from(a, 'base64url')).toHaveLength(48);
    expect(newSecret()).not.toBe(a);
    expect(() => newSecret(() => Buffer.alloc(16))).toThrow(/no secret/);
  });

  it('Create: the edge copy first (never over an existing one), then the instance copy; the same value, SecureString; no value in the answer or the log', async () => {
    const f = fakeSsm();
    const { response, logs } = await run(originSecret, event('Create'), { ssm: f.ssm });
    expect(response).toMatchObject({ Status: 'SUCCESS', PhysicalResourceId: PROPS.EdgeParameter, NoEcho: true, RequestId: 'req-1' });
    expect(response.Data).toBeUndefined();
    expect(f.calls.map((c) => [c.region, c.op, c.input.Name, c.input.Type, c.input.Overwrite])).toEqual([
      ['us-east-1', 'put', PROPS.EdgeParameter, 'SecureString', false],
      ['eu-central-1', 'put', PROPS.SecretParameter, 'SecureString', true],
    ]);
    const value = f.params[`us-east-1 ${PROPS.EdgeParameter}`];
    expect(value).toMatch(/^[A-Za-z0-9_-]{64}$/);
    expect(f.params[`eu-central-1 ${PROPS.SecretParameter}`]).toBe(value);
    expect(f.calls.every((c) => c.input.KeyId === undefined)).toBe(true); // aws/ssm
    expect(JSON.stringify(response) + logs.join('\n')).not.toContain(value);
  });

  it('Create with both parameters there (the installer made them before the deployment) keeps them as they are', async () => {
    const existing = { [`us-east-1 ${PROPS.EdgeParameter}`]: 'installers', [`eu-central-1 ${PROPS.SecretParameter}`]: 'installers' };
    const f = fakeSsm({ existing });
    const { response, logs } = await run(originSecret, event('Create'), { ssm: f.ssm });
    expect(response).toMatchObject({ Status: 'SUCCESS', PhysicalResourceId: PROPS.EdgeParameter });
    expect(f.params).toEqual(existing);
    expect(f.calls.map((c) => [c.region, c.op, c.input.Overwrite])).toEqual([['us-east-1', 'put', false], ['eu-central-1', 'put', false]]);
    expect(logs.join('\n')).toContain('"written":false');
  });

  it('Create over another installation\'s edge copy (no instance copy here) fails, leaves the parameters as they were, and its rollback deletes nothing', async () => {
    const f = fakeSsm({ existing: { [`us-east-1 ${PROPS.EdgeParameter}`]: 'theirs' } });
    const { response } = await run(originSecret, event('Create'), { ssm: f.ssm });
    expect(response.Status).toBe('FAILED');
    expect(response.Reason).toMatch(/already exists in us-east-1 without \/factoryos\/factoryos\/origin-secret/);
    expect(response.PhysicalResourceId).not.toBe(PROPS.EdgeParameter);
    expect(f.params).toEqual({ [`us-east-1 ${PROPS.EdgeParameter}`]: 'theirs' });
    // The only delete: of the instance copy the create itself had just made.
    expect(f.calls.filter((c) => c.op === 'del').map((c) => [c.region, c.input.Name])).toEqual([['eu-central-1', PROPS.SecretParameter]]);
    const before = f.calls.length;
    const rollback = await run(originSecret, event('Delete', { PhysicalResourceId: response.PhysicalResourceId }), { ssm: f.ssm });
    expect(rollback.response.Status).toBe('SUCCESS');
    expect(f.calls.slice(before)).toEqual([]);
    expect(f.params).toEqual({ [`us-east-1 ${PROPS.EdgeParameter}`]: 'theirs' });
  });

  it('Create: when the instance copy fails, the edge copy it just wrote is removed', async () => {
    const f = fakeSsm({ fail: { 'eu-central-1 put': 'AccessDeniedException' } });
    const { response } = await run(originSecret, event('Create'), { ssm: f.ssm });
    expect(response.Status).toBe('FAILED');
    expect(response.Reason).toMatch(/^AccessDeniedException: /);
    expect(f.params).toEqual({});
  });

  it('Update changes nothing (rotation is the installer\'s)', async () => {
    const f = fakeSsm({ existing: { [`us-east-1 ${PROPS.EdgeParameter}`]: 'v', [`eu-central-1 ${PROPS.SecretParameter}`]: 'v' } });
    const { response } = await run(originSecret, event('Update', { PhysicalResourceId: PROPS.EdgeParameter, OldResourceProperties: PROPS }), { ssm: f.ssm });
    expect(response).toMatchObject({ Status: 'SUCCESS', PhysicalResourceId: PROPS.EdgeParameter });
    expect(f.calls).toEqual([]);
  });

  it('Delete of its own: both parameters (already gone is fine); another error fails; bad properties delete nothing', async () => {
    const f = fakeSsm({ existing: { [`us-east-1 ${PROPS.EdgeParameter}`]: 'v' } });
    const { response } = await run(originSecret, event('Delete', { PhysicalResourceId: PROPS.EdgeParameter }), { ssm: f.ssm });
    expect(response.Status).toBe('SUCCESS');
    expect(f.calls.map((c) => [c.region, c.op, c.input.Name])).toEqual([
      ['us-east-1', 'del', PROPS.EdgeParameter], ['eu-central-1', 'del', PROPS.SecretParameter],
    ]);
    expect(f.params).toEqual({});
    const g = fakeSsm({ fail: { 'us-east-1 del': 'ThrottlingException' } });
    expect((await run(originSecret, event('Delete', { PhysicalResourceId: PROPS.EdgeParameter }), { ssm: g.ssm })).response.Status).toBe('FAILED');
    const h = fakeSsm();
    const bad = event('Delete', { PhysicalResourceId: PROPS.EdgeParameter, ResourceProperties: { ...PROPS, SecretParameter: '/other' } });
    expect((await run(originSecret, bad, { ssm: h.ssm })).response.Status).toBe('SUCCESS');
    expect(h.calls).toEqual([]);
  });

  it('accepts only this stack\'s two parameter names', () => {
    expect(SECRET_PARAMETER.test('/factoryos/demo-1/origin-secret')).toBe(true);
    expect(EDGE_PARAMETER.test('/factoryos/demo-1/edge-origin-secret')).toBe(true);
    for (const bad of [
      { SecretParameter: '/factoryos/a/release/wanted' },
      { EdgeParameter: '/factoryos/factoryos/edge-config' },
      { EdgeParameter: '/factoryos/other/edge-origin-secret' },
      { EdgeRegion: 'us-east-1a' },
    ]) {
      expect(() => propsOf({ ...PROPS, ...bad }), JSON.stringify(bad)).toThrow();
    }
  });
});

describe('VpcOriginSecurityGroupLookup', () => {
  const props = { VpcId: 'vpc-0123456789abcdef0', GroupName: 'CloudFront-VPCOrigins-Service-SG' };
  const ev = (RequestType, extra = {}) => ({ ...event(RequestType, extra), LogicalResourceId: 'VpcOriginSecurityGroupLookup', ResourceProperties: props });

  it('answers the group of this VPC with that name (attribute GroupId, physical id)', async () => {
    const inputs = [];
    const describe = async (input) => { inputs.push(input); return { SecurityGroups: [{ GroupId: 'sg-0abc' }] }; };
    const { response } = await run(sgLookup, ev('Create'), { describe });
    expect(response).toMatchObject({ Status: 'SUCCESS', PhysicalResourceId: 'sg-0abc', Data: { GroupId: 'sg-0abc' } });
    expect(inputs).toEqual([{ Filters: [{ Name: 'vpc-id', Values: [props.VpcId] }, { Name: 'group-name', Values: [props.GroupName] }] }]);
  });

  it('retries a missing group, then fails; two groups fail at once; Delete calls nothing', async () => {
    let n = 0;
    const late = async () => (++n < 3 ? { SecurityGroups: [] } : { SecurityGroups: [{ GroupId: 'sg-1' }] });
    expect(await lookup({ vpcId: props.VpcId, groupName: props.GroupName }, { describe: late, sleep: async () => {} })).toBe('sg-1');
    const never = await run(sgLookup, ev('Update', { PhysicalResourceId: 'sg-1' }), { describe: async () => ({ SecurityGroups: [] }), sleep: async () => {} });
    expect(never.response).toMatchObject({ Status: 'FAILED', PhysicalResourceId: 'sg-1' });
    const two = await run(sgLookup, ev('Create'), { describe: async () => ({ SecurityGroups: [{ GroupId: 'sg-1' }, { GroupId: 'sg-2' }] }) });
    expect(two.response.Reason).toMatch(/2 security groups/);
    let called = false;
    const del = await run(sgLookup, ev('Delete', { PhysicalResourceId: 'sg-1' }), { describe: async () => { called = true; return {}; } });
    expect(del.response).toMatchObject({ Status: 'SUCCESS', PhysicalResourceId: 'sg-1' });
    expect(called).toBe(false);
  });
});

describe('the denial matrix: a client-sent x-origin-verify', () => {
  const f = gateFixture();
  const props = { base: `https://${f.cfg.distributionDomain}`, clientId: f.cfg.clientId, cognitoDomain: f.cfg.cognitoDomain, issuer: f.issuer, region: f.cfg.region };

  it('is one of the checks, and gets the gate\'s 401 like any anonymous request', async () => {
    const c = matrix(props).find((x) => x.name === 'API GET with a client-sent x-origin-verify');
    expect(c.request.headers['x-origin-verify']).toMatch(/^[A-Za-z0-9_-]{64}$/);
    const r = await f.gate().handler(f.event({ method: 'GET', uri: c.request.path, headers: c.request.headers }));
    expect(String(r.status)).toBe('401');
    // A gate that let it through (answering 200) fails the whole check at once.
    const open = await runMatrix(props, async (req) => (req.headers['x-origin-verify'] ? { status: 200, headers: {} } : { status: 401, headers: { 'www-authenticate': 'FactoryOS realm="factoryos"' } }));
    expect(open.fatal).toMatch(/client-sent x-origin-verify: answered 200/);
  });
});
