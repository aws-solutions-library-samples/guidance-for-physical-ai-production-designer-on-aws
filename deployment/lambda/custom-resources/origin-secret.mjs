// FactoryOS public deployment: the OriginSecret custom resource (CloudFormation calls this Lambda directly).
//
// The origin secret (deployment/CONTRACT.md "Origin secret"): 48 random bytes, base64url, generated here at deploy
// time, so that no template, parameter, user data or custom resource property ever carries it. It is written to two
// SecureString parameters (the AWS managed key aws/ssm):
//   - EdgeParameter in EdgeRegion (us-east-1), /factoryos/<stack>/edge-origin-secret: the edge function's copy, which
//     it sends to the origin as x-origin-verify;
//   - SecretParameter in the stack's Region, /factoryos/<stack>/origin-secret: the instance's copy (the release writes
//     it into the backend's EnvironmentFile as FACTORY_OS_ORIGIN_SECRET).
// Create (a new stack, or the first update of an installation from before the origin secret): a new secret into both,
// the edge copy first; both already there (the installer's, made before the deployment): kept; the edge copy alone
// there (the same stack name in another Region of this account: that installation's secret): never overwritten, the
// deploy fails instead, as EdgeConfig's does. Update: nothing
// (the resource's properties are names; rotation is the installer's, --rotate-origin-secret, which writes the two
// parameters itself). Delete: both parameters, only when the physical id is this resource's own (a failed create
// answers another one, so its rollback deletes nothing).
// The value is never logged, never returned (no Data, NoEcho) and never part of an error message.

import crypto from 'node:crypto';
import { describeError, responseBase, sendResponse } from './cfn-response.mjs';

export const SECRET_BYTES = 48;
const STACK = '[a-z][a-z0-9-]{0,30}';
const REGION = /^[a-z]{2}(-[a-z]+)+-\d$/;
export const SECRET_PARAMETER = new RegExp(`^/factoryos/${STACK}/origin-secret$`);
export const EDGE_PARAMETER = new RegExp(`^/factoryos/${STACK}/edge-origin-secret$`);

/** A new secret: SECRET_BYTES random bytes, base64url (64 characters, no padding). */
export function newSecret(random = crypto.randomBytes) {
  const bytes = random(SECRET_BYTES);
  if (!Buffer.isBuffer(bytes) || bytes.length !== SECRET_BYTES) throw new Error('the random source returned no secret');
  return bytes.toString('base64url');
}

/** The resource's properties, validated. */
export function propsOf(p = {}) {
  const out = {
    secretParameter: String(p.SecretParameter || ''),
    secretRegion: String(p.SecretRegion || ''),
    edgeParameter: String(p.EdgeParameter || ''),
    edgeRegion: String(p.EdgeRegion || ''),
  };
  if (!SECRET_PARAMETER.test(out.secretParameter)) throw new Error('SecretParameter is not /factoryos/<stack>/origin-secret');
  if (!EDGE_PARAMETER.test(out.edgeParameter)) throw new Error('EdgeParameter is not /factoryos/<stack>/edge-origin-secret');
  if (out.secretParameter.split('/')[2] !== out.edgeParameter.split('/')[2]) throw new Error('the parameters name different stacks');
  if (!REGION.test(out.secretRegion) || !REGION.test(out.edgeRegion)) throw new Error('SecretRegion or EdgeRegion is not a Region');
  return out;
}

async function sdkSsm(region) {
  const { SSMClient, PutParameterCommand, DeleteParameterCommand } = await import('@aws-sdk/client-ssm');
  const client = new SSMClient({ region });
  return {
    put: (input) => client.send(new PutParameterCommand(input)),
    del: (input) => client.send(new DeleteParameterCommand(input)),
  };
}

const description = (props, which) => `FactoryOS ${props.secretParameter.split('/')[2]}: the origin secret (${which}; `
  + 'written by the stack\'s OriginSecret custom resource, deployment/CONTRACT.md "Origin secret")';

/**
 * Create: the parameters as a pair. Neither exists: both get the new secret. Both exist (the installer made them
 * before the deployment, deployment/installer/origin.sh, or a create repeated): kept as they are. Only the edge copy
 * exists: it is not this installation's (the same stack name in another Region of this account, or an uninstall's
 * leftover) and is never overwritten: the deploy fails, as EdgeConfig's does. No call reads a value back (the role
 * may not read either parameter); existence is learned from PutParameter without Overwrite.
 * Returns 'created' or 'kept'.
 */
async function create(props, ssm, secret) {
  const edge = await ssm(props.edgeRegion);
  const instance = await ssm(props.secretRegion);
  const put = (client, name, overwrite, which) => client.put({
    Name: name, Value: secret, Type: 'SecureString', Tier: 'Standard', Overwrite: overwrite, Description: description(props, which),
  });
  const exists = (e) => e?.name === 'ParameterAlreadyExists';
  let edgeNew = true;
  try {
    await put(edge, props.edgeParameter, false, 'the edge function\'s copy');
  } catch (e) {
    if (!exists(e)) throw e;
    edgeNew = false;
  }
  try {
    // With a new edge copy, over whatever the instance's name holds (in this Region the name is this stack's: a
    // leftover can only be a previous installation's); with an existing one, only if there is none.
    await put(instance, props.secretParameter, edgeNew, 'the instance\'s copy');
  } catch (e) {
    if (edgeNew) {
      // A failed create leaves nothing behind (its rollback deletes nothing: the physical id is not this resource's own).
      await edge.del({ Name: props.edgeParameter }).catch(() => {});
      throw e;
    }
    if (exists(e)) return 'kept';
    throw e;
  }
  if (edgeNew) return 'created';
  // The edge copy existed, the instance's did not: undo the instance's and refuse.
  await instance.del({ Name: props.secretParameter }).catch(() => {});
  throw new Error(`${props.edgeParameter} already exists in ${props.edgeRegion} without ${props.secretParameter} in ${props.secretRegion}: `
    + `another installation of the stack name ${props.secretParameter.split('/')[2]} in this account (in another Region) uses it, `
    + 'or an uninstall left it behind; install with another --stack, or delete the leftover');
}

async function remove(props, ssm) {
  for (const [region, name] of [[props.edgeRegion, props.edgeParameter], [props.secretRegion, props.secretParameter]]) {
    try {
      await (await ssm(region)).del({ Name: name });
    } catch (e) {
      if (e?.name !== 'ParameterNotFound') throw e;
    }
  }
}

/** The custom resource's work; `send` answers CloudFormation, `ssm(region)` is { put, del } of SSM. */
export async function handle(event, _context, { send = sendResponse, ssm = sdkSsm, random = crypto.randomBytes, log = console.log } = {}) {
  const failedId = `origin-secret-not-created-${event.RequestId}`;
  let physicalId = event.PhysicalResourceId || failedId;
  let written = false;
  let result;
  let props = null;
  try {
    props = propsOf(event.ResourceProperties);
  } catch (e) {
    // A Delete whose properties do not validate deletes nothing and succeeds (it must not block the uninstall).
    result = event.RequestType === 'Delete'
      ? { ...responseBase(event, physicalId), Status: 'SUCCESS', NoEcho: true }
      : { ...responseBase(event, physicalId), Status: 'FAILED', Reason: describeError(e) };
  }
  if (props) {
    try {
      const ownId = props.edgeParameter;
      if (event.RequestType === 'Create') {
        written = (await create(props, ssm, newSecret(random))) === 'created';
        physicalId = ownId;
      } else if (event.RequestType === 'Delete') {
        if (physicalId === ownId) await remove(props, ssm);
      }
      result = { ...responseBase(event, physicalId), Status: 'SUCCESS', NoEcho: true };
    } catch (e) {
      result = { ...responseBase(event, physicalId), Status: 'FAILED', Reason: describeError(e) };
    }
  }
  log(JSON.stringify({ request: event.RequestType, status: result.Status, written, reason: result.Reason }));
  await send(event, result);
}

export const handler = (event, context) => handle(event, context);
