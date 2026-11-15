// FactoryOS public deployment: the VpcOriginSecurityGroupLookup custom resource (CloudFormation calls this Lambda
// directly). It finds the security group CloudFront creates in the stack's VPC for its VPC origins
// (CloudFront-VPCOrigins-Service-SG) and answers its id (attribute GroupId, also the physical id): the source of the
// instance's only ingress rule (OriginIngress), which the stack adds after VerifyAnonymousDenial passed.
// Read only (ec2:DescribeSecurityGroups); Delete does nothing. A function and role of its own: no other custom
// resource runs with this role, and this one runs with no other permission.

import { describeError, responseBase, sendResponse } from './cfn-response.mjs';

const VPC_ID = /^vpc-[0-9a-f]{8,17}$/;
const GROUP_NAME = /^[A-Za-z0-9 ._-]{1,255}$/;
const ATTEMPTS = 6;
const RETRY_MS = 10 * 1000;

export function propsOf(p = {}) {
  const vpcId = String(p.VpcId || '');
  const groupName = String(p.GroupName || '');
  if (!VPC_ID.test(vpcId)) throw new Error('VpcId is not a VPC id');
  if (!GROUP_NAME.test(groupName)) throw new Error('GroupName is not a security group name');
  return { vpcId, groupName };
}

async function sdkDescribe(input) {
  const { EC2Client, DescribeSecurityGroupsCommand } = await import('@aws-sdk/client-ec2');
  return new EC2Client({}).send(new DescribeSecurityGroupsCommand(input));
}

/** The group's id; retried briefly (CloudFront creates the group with the VPC origin). Exactly one, or an error. */
export async function lookup({ vpcId, groupName }, { describe = sdkDescribe, sleep = (ms) => new Promise((r) => setTimeout(r, ms)) } = {}) {
  for (let attempt = 1; ; attempt++) {
    const r = await describe({ Filters: [{ Name: 'vpc-id', Values: [vpcId] }, { Name: 'group-name', Values: [groupName] }] });
    const groups = r?.SecurityGroups || [];
    if (groups.length > 1) throw new Error(`${groups.length} security groups named ${groupName} in ${vpcId}`);
    if (groups.length === 1 && /^sg-[0-9a-f]+$/.test(groups[0].GroupId || '')) return groups[0].GroupId;
    if (attempt >= ATTEMPTS) throw new Error(`no security group ${groupName} in ${vpcId} (CloudFront creates it with the VPC origin)`);
    await sleep(RETRY_MS);
  }
}

export async function handle(event, _context, { send = sendResponse, describe, sleep, log = console.log } = {}) {
  const physicalId = event.PhysicalResourceId || `vpc-origin-sg-not-found-${event.RequestId}`;
  let result;
  if (event.RequestType === 'Delete') {
    result = { ...responseBase(event, physicalId), Status: 'SUCCESS' };
  } else {
    try {
      const groupId = await lookup(propsOf(event.ResourceProperties), { ...(describe ? { describe } : {}), ...(sleep ? { sleep } : {}) });
      result = { ...responseBase(event, groupId), Status: 'SUCCESS', Data: { GroupId: groupId } };
    } catch (e) {
      result = { ...responseBase(event, physicalId), Status: 'FAILED', Reason: describeError(e) };
    }
  }
  log(JSON.stringify({ request: event.RequestType, status: result.Status, groupId: result.Data?.GroupId, reason: result.Reason }));
  await send(event, result);
}

export const handler = (event, context) => handle(event, context);
