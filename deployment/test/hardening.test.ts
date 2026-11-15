// The review fixes of the CDK app: the flow log role (confused-deputy conditions), one role per custom resource, the
// origin secret's CDK side (no value in any template, who may read and write it), Cognito refresh-token rotation, and
// a retained log group of the stack for every Lambda function of both stacks.
import * as fs from 'node:fs';
import * as os from 'node:os';
import * as path from 'node:path';
import { App } from 'aws-cdk-lib';
import { describe, expect, it } from 'vitest';
import { buildApp } from '../lib/app';
import { edgeOriginSecretParameterName } from '../lib/edge';
import { FLOW_LOG_ROLE_ID, originSecretParameterName, PINNED_IDS, REFRESH_ROTATION_GRACE } from '../lib/factoryos-stack';

const ENV = { CDK_DEFAULT_ACCOUNT: '111122223333', CDK_DEFAULT_REGION: 'eu-central-1' };
const BASE = { amiId: 'ami-0123456789abcdef0' };

interface Synth { main: Record<string, any>; edge: Record<string, any> }

function synth(context: Record<string, unknown> = {}, env: Record<string, string> = ENV): Synth {
  const app = new App({ outdir: fs.mkdtempSync(path.join(os.tmpdir(), 'factoryos-cdk-')), context: { ...BASE, ...context } });
  const { main, edge } = buildApp(app, env);
  const assembly = app.synth();
  return { main: assembly.getStackByName(main.stackName).template, edge: assembly.getStackByName(edge.stackName).template };
}

const base = synth();
const east = synth({}, { ...ENV, CDK_DEFAULT_REGION: 'us-east-1' });
const all = synth({ bedrock: 'true', kiroSecretName: 'factoryos/kiro', detailedMonitoring: 'true' });  // pragma: allowlist secret (a secret's name, not its value)
const VARIANTS: Array<[string, Synth]> = [['eu-central-1', base], ['us-east-1', east], ['every option', all]];

const list = (v: unknown): any[] => (Array.isArray(v) ? v : [v]);
const ofType = (t: Record<string, any>, type: string) => Object.entries<any>(t.Resources).filter(([, r]) => r.Type === type);
/** A resource of a statement as text, `${AWS::Partition}` for the partition. */
const arnText = (r: any): string => (typeof r === 'string' ? r : r['Fn::Join'] ? r['Fn::Join'][1].map((x: any) => (typeof x === 'string' ? x
  : x.Ref ? `\${${x.Ref}}` : `<${JSON.stringify(x)}>`)).join('') : `<${JSON.stringify(r)}>`);
/** The logical id a GetAtt/Ref names. */
const target = (v: any): string => v?.['Fn::GetAtt']?.[0] ?? v?.Ref;
/** Every statement a role has: inline (Policies) and of the AWS::IAM::Policy resources attached to it. */
function roleStatements(t: Record<string, any>, roleId: string): any[] {
  const role = t.Resources[roleId];
  const inline = (role.Properties.Policies || []).flatMap((p: any) => p.PolicyDocument.Statement);
  const attached = ofType(t, 'AWS::IAM::Policy')
    .filter(([, p]) => p.Properties.Roles.some((r: any) => r.Ref === roleId))
    .flatMap(([, p]) => p.Properties.PolicyDocument.Statement);
  return [...inline, ...attached];
}
/** custom resource id -> its function's role id (the Lambda its ServiceToken names). */
function customResourceRoles(t: Record<string, any>): Array<[string, string]> {
  return Object.entries<any>(t.Resources)
    .filter(([, r]) => r.Type.startsWith('Custom::') || r.Type === 'AWS::CloudFormation::CustomResource')
    .map(([id, r]) => {
      const fn = t.Resources[target(r.Properties.ServiceToken)];
      expect(fn?.Type, id).toBe('AWS::Lambda::Function');
      return [id, target(fn.Properties.Role)];
    });
}

describe('(1) the VPC flow log role', () => {
  it('is assumed by vpc-flow-logs.amazonaws.com only, for this account\'s flow logs in this Region (aws:SourceAccount, aws:SourceArn)', () => {
    const [, flowLog] = ofType(base.main, 'AWS::EC2::FlowLog')[0];
    expect(ofType(base.main, 'AWS::EC2::FlowLog')).toHaveLength(1);
    expect(flowLog.Properties.DeliverLogsPermissionArn).toEqual({ 'Fn::GetAtt': [FLOW_LOG_ROLE_ID, 'Arn'] });
    const role = base.main.Resources[FLOW_LOG_ROLE_ID];
    expect(role.Type).toBe('AWS::IAM::Role');
    expect(role.Properties.ManagedPolicyArns).toBeUndefined();
    expect(role.Properties.Policies).toBeUndefined();
    const trust = role.Properties.AssumeRolePolicyDocument.Statement;
    expect(trust).toHaveLength(1);
    expect(trust[0]).toMatchObject({ Action: 'sts:AssumeRole', Effect: 'Allow', Principal: { Service: 'vpc-flow-logs.amazonaws.com' } });
    expect(Object.keys(trust[0].Condition).sort()).toEqual(['ArnLike', 'StringEquals']);
    expect(trust[0].Condition.StringEquals).toEqual({ 'aws:SourceAccount': '111122223333' });
    expect(Object.keys(trust[0].Condition.ArnLike)).toEqual(['aws:SourceArn']);
    expect(arnText(trust[0].Condition.ArnLike['aws:SourceArn'])).toBe('arn:${AWS::Partition}:ec2:eu-central-1:111122223333:vpc-flow-log/*');
  });

  it('may write to its log group only', () => {
    const [, flowLog] = ofType(base.main, 'AWS::EC2::FlowLog')[0];
    const group = flowLog.Properties.LogGroupName.Ref;
    expect(base.main.Resources[group].Properties.RetentionInDays).toBe(30);
    expect(roleStatements(base.main, FLOW_LOG_ROLE_ID)).toEqual([{
      Action: ['logs:CreateLogStream', 'logs:PutLogEvents', 'logs:DescribeLogStreams'],
      Effect: 'Allow',
      Resource: { 'Fn::GetAtt': [group, 'Arn'] },
    }]);
  });

  it('keeps the logical id of the role CDK created before (an update changes it in place, the flow log is not replaced)', () => {
    expect(FLOW_LOG_ROLE_ID).toBe('VpcallIAMRole2920156A');
    // No other role trusts the flow logs service.
    const trusting = ofType(base.main, 'AWS::IAM::Role').filter(([, r]) => JSON.stringify(r.Properties.AssumeRolePolicyDocument).includes('vpc-flow-logs'));
    expect(trusting.map(([id]) => id)).toEqual([FLOW_LOG_ROLE_ID]);
  });
});

describe('(2) every custom resource runs with a role of its own', () => {
  it('no two custom resources share a role, no two functions share a role (both stacks, every variant)', () => {
    for (const [name, s] of VARIANTS) {
      for (const t of [s.main, s.edge]) {
        const roles = customResourceRoles(t).map(([, role]) => role);
        expect(new Set(roles).size, `${name}: ${JSON.stringify(customResourceRoles(t))}`).toBe(roles.length);
        const fnRoles = ofType(t, 'AWS::Lambda::Function').map(([, f]) => target(f.Properties.Role));
        expect(new Set(fnRoles).size, name).toBe(fnRoles.length);
      }
    }
    expect(customResourceRoles(base.main).map(([id]) => id).sort()).toEqual([
      'AccessLogBucketAutoDeleteObjectsCustomResource01AB31E8', 'EdgeAuthArnReader7A37DB80', 'EdgeConfig3DA27A0B', 'OriginSecret',
      PINNED_IDS.verify, 'VpcOriginSecurityGroupLookup',
    ].sort());
  });

  it('each role has its custom resource\'s permissions only', () => {
    const roles = Object.fromEntries(customResourceRoles(base.main));
    const actions = (id: string) => roleStatements(base.main, roles[id]).map((st) => [list(st.Action).sort(), list(st.Resource).map(arnText)]);
    expect(actions('EdgeConfig3DA27A0B')).toEqual([[['ssm:DeleteParameter', 'ssm:PutParameter'],
      ['arn:${AWS::Partition}:ssm:us-east-1:111122223333:parameter/factoryos/factoryos/edge-config']]]);
    expect(actions('VpcOriginSecurityGroupLookup')).toEqual([[['ec2:DescribeSecurityGroups'], ['*']]]);
    expect(actions('OriginSecret')).toEqual([[['ssm:DeleteParameter', 'ssm:PutParameter'], [
      'arn:${AWS::Partition}:ssm:eu-central-1:111122223333:parameter/factoryos/factoryos/origin-secret',
      'arn:${AWS::Partition}:ssm:us-east-1:111122223333:parameter/factoryos/factoryos/edge-origin-secret',
    ]]]);
    expect(actions(PINNED_IDS.verify)).toEqual([]);
    // The security group lookup: no longer a Custom::AWS of the AwsCustomResource singleton; EdgeConfig is its only one.
    expect(ofType(base.main, 'Custom::AWS').map(([id]) => id)).toEqual(['EdgeConfig3DA27A0B']);
    expect(base.main.Resources.VpcOriginSecurityGroupLookup.Type).toBe('Custom::VpcOriginSecurityGroup');
  });
});

describe('(3) the origin secret, CDK side', () => {
  const secretName = originSecretParameterName('factoryos');
  const edgeName = edgeOriginSecretParameterName('factoryos');

  it('is generated by the OriginSecret custom resource: its properties are names, nothing reads a value back', () => {
    const r = base.main.Resources.OriginSecret;
    expect(r.Type).toBe('Custom::OriginSecret');
    const { ServiceToken, ServiceTimeout, ...props } = r.Properties;
    expect(props).toEqual({ SecretParameter: secretName, SecretRegion: 'eu-central-1', EdgeParameter: edgeName, EdgeRegion: 'us-east-1' }); // pragma: allowlist secret (property names; Region names)
    expect(base.main.Resources[target(ServiceToken)].Properties.Handler).toBe('origin-secret.handler');
    // No attribute of it is used anywhere, and CloudFormation never holds the secret as a parameter of its own.
    for (const s of [base, east]) {
      for (const t of [s.main, s.edge]) {
        expect(JSON.stringify(t)).not.toMatch(/"Fn::GetAtt":\["OriginSecret",/);
        expect(ofType(t, 'AWS::SSM::Parameter').map(([, p]) => p.Properties.Name)).not.toContain(secretName);
        expect(ofType(t, 'AWS::SSM::Parameter').map(([, p]) => p.Properties.Name)).not.toContain(edgeName);
        expect(JSON.stringify(t.Parameters || {})).not.toContain('origin-secret');
        // Only the release handoff's text (it checks the parameter's type) names a SecureString: no resource is one.
        const others = Object.entries<any>(t.Resources).filter(([id]) => id !== PINNED_IDS.releaseDocument);
        expect(JSON.stringify(others)).not.toContain('SecureString');
      }
    }
  });

  it('exists before the distribution associates the edge version that sends it', () => {
    expect(base.main.Resources[PINNED_IDS.distribution].DependsOn).toContain('OriginSecret');
    expect(east.main.Resources[PINNED_IDS.distribution].DependsOn).toContain('OriginSecret');
  });

  it('the instance learns the name through the release document, never the user data; only the instance and edge roles may read', () => {
    const run = JSON.stringify(base.main.Resources[PINNED_IDS.releaseDocument].Properties.Content.mainSteps[0].inputs.runCommand);
    expect(run).toContain(`export FACTORYOS_ORIGIN_SECRET_PARAMETER='${secretName}'`);
    const userData = JSON.stringify(base.main.Resources[PINNED_IDS.instance].Properties.UserData);
    expect(userData).not.toMatch(/ORIGIN_SECRET|origin-secret/);
    for (const [name, s] of VARIANTS) {
      const readers: string[] = [];
      for (const [label, t] of [['main', s.main], ['edge', s.edge]] as const) {
        for (const [roleId] of ofType(t, 'AWS::IAM::Role')) {
          for (const st of roleStatements(t, roleId)) {
            const res = list(st.Resource).map(arnText).join(' ');
            if (!/origin-secret/.test(res)) continue;
            const acts = list(st.Action);
            if (acts.some((a: string) => a.startsWith('ssm:Get'))) readers.push(`${label}:${roleId}:${res}`);
          }
        }
      }
      expect(readers.length, name).toBe(2);
      expect(readers.find((x) => x.startsWith(`main:${PINNED_IDS.instanceRole}:`)), name)
        .toBe(`main:${PINNED_IDS.instanceRole}:arn:\${AWS::Partition}:ssm:${name === 'us-east-1' ? 'us-east-1' : 'eu-central-1'}:111122223333:parameter${secretName}`);
      const edgeReader = readers.find((x) => x.startsWith('edge:'))!;
      expect(edgeReader).toMatch(/^edge:EdgeAuth\w*ServiceRole/);
      expect(edgeReader).toContain(`:ssm:us-east-1:111122223333:parameter${edgeName}`);
      expect(edgeReader).not.toContain(`parameter${secretName}`);
    }
  });

  it('the edge role reads its configuration and the edge copy only, with ssm:GetParameter', () => {
    const [roleId] = ofType(base.edge, 'AWS::IAM::Role').find(([id]) => /^EdgeAuth\w*ServiceRole/.test(id))!;
    const st = roleStatements(base.edge, roleId);
    expect(st).toHaveLength(1);
    expect(st[0].Action).toBe('ssm:GetParameter');
    expect(list(st[0].Resource).map(arnText)).toEqual([
      'arn:${AWS::Partition}:ssm:us-east-1:111122223333:parameter/factoryos/factoryos/edge-config',
      `arn:\${AWS::Partition}:ssm:us-east-1:111122223333:parameter${edgeName}`,
    ]);
  });
});

describe('(4) Cognito refresh-token rotation', () => {
  it('is on, with the grace period, without REFRESH_TOKEN_AUTH, and the token validities unchanged', () => {
    for (const [, s] of VARIANTS) {
      const c = s.main.Resources[PINNED_IDS.userPoolClient].Properties;
      expect(c.RefreshTokenRotation).toEqual({ Feature: 'ENABLED', RetryGracePeriodSeconds: REFRESH_ROTATION_GRACE.toSeconds() });
      expect(REFRESH_ROTATION_GRACE.toSeconds()).toBeLessThanOrEqual(60);
      expect(c.ExplicitAuthFlows).toEqual(['ALLOW_USER_SRP_AUTH']);
      expect([c.AccessTokenValidity, c.IdTokenValidity, c.RefreshTokenValidity]).toEqual([60, 60, 720]);
      expect(c.TokenValidityUnits).toEqual({ AccessToken: 'minutes', IdToken: 'minutes', RefreshToken: 'minutes' });
      expect(c.EnableTokenRevocation).toBe(true);
    }
  });
});

describe('(5) every Lambda function logs to a log group of its stack with a retention', () => {
  it('LoggingConfig.LogGroup names a log group of the same template: 30 days, deleted with the stack (both stacks, every variant)', () => {
    for (const [name, s] of VARIANTS) {
      for (const t of [s.main, s.edge]) {
        const fns = ofType(t, 'AWS::Lambda::Function');
        expect(fns.length).toBeGreaterThan(0);
        const groups = fns.map(([id, f]) => {
          const ref = f.Properties.LoggingConfig?.LogGroup;
          expect(ref?.Ref, `${name} ${id}`).toBeDefined();
          const g = t.Resources[ref.Ref];
          expect(g?.Type, `${name} ${id}`).toBe('AWS::Logs::LogGroup');
          expect(g.Properties.RetentionInDays, `${name} ${id}`).toBe(30);
          expect(g.DeletionPolicy, `${name} ${id}`).toBe('Delete');
          return ref.Ref;
        });
        // One group per function, except the AwsCustomResource singleton's (EdgeConfig only), which is unshared now too.
        expect(new Set(groups).size, name).toBe(groups.length);
      }
    }
  });

  it('the edge function: a CloudFormation-named group in us-east-1, the name its replicas use in every Region', () => {
    const [, fn] = ofType(base.edge, 'AWS::Lambda::Function')[0];
    const group = base.edge.Resources[fn.Properties.LoggingConfig.LogGroup.Ref];
    expect(group.Properties.LogGroupName).toBeUndefined();
    expect(fn.Properties.LoggingConfig).toEqual({ LogGroup: { Ref: fn.Properties.LoggingConfig.LogGroup.Ref } });
    // CDK's own provider functions (no lambda.Function construct): a group each too.
    for (const id of ['CustomS3AutoDeleteObjectsCustomResourceProviderHandler9D90184F', 'CustomCrossRegionStringParameterReaderCustomResourceProviderHandler65B5F33A']) { // pragma: allowlist secret (CDK logical IDs)
      expect(base.main.Resources[id].Properties.LoggingConfig.LogGroup.Ref, id).toMatch(/Logs/);
    }
  });
});

describe('the instance boots with outbound internet', () => {
  it('depends on the NAT default route of every private subnet (both Regions, every variant)', () => {
    for (const [name, s] of VARIANTS) {
      const t = s.main;
      const natRoutes = ofType(t, 'AWS::EC2::Route').filter(([, r]) => r.Properties.NatGatewayId).map(([id]) => id);
      expect(natRoutes.length, name).toBeGreaterThan(0);
      const deps: string[] = t.Resources[PINNED_IDS.instance].DependsOn ?? [];
      for (const id of natRoutes) expect(deps, `${name}: ${id}`).toContain(id);
    }
  });

});
