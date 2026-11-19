// The CDK app: names and outputs of deployment/CONTRACT.md, stability of what an update must never replace, the
// deploy order (origin empty -> gate -> verify -> open), and cdk-nag (a synth with an unacknowledged finding fails).
import * as fs from 'node:fs';
import * as os from 'node:os';
import * as path from 'node:path';
import { App } from 'aws-cdk-lib';
import { Template } from 'aws-cdk-lib/assertions';
import * as iam from 'aws-cdk-lib/aws-iam';
import { describe, expect, it } from 'vitest';
import { buildApp, INSTALLER_FORMAT_TAG, SOLUTION_ID, SOLUTION_ID_FORMAT, stackDescription } from '../lib/app';
import { defaultBedrockModelIds, inferenceProfilePrefix, readConfig } from '../lib/config';
import { edgeConfigParameterName, EDGE_SOURCE_DIR } from '../lib/edge';
import { AGENT_METRICS_NAMESPACE, domainPrefix, EDGE_CONFIG_DELETE_IGNORED, PINNED_IDS, readBootstrap } from '../lib/factoryos-stack';

const ENV = { CDK_DEFAULT_ACCOUNT: '111122223333', CDK_DEFAULT_REGION: 'eu-central-1' };
const BASE = { amiId: 'ami-0123456789abcdef0' };

interface Synth {
  main: Template;
  edge: Template;
  json: Record<string, any>;
  edgeJson: Record<string, any>;
  outdir: string;
  mainStackName: string;
  edgeStackName: string;
  tags: Record<string, Record<string, string>>;
}

function synth(context: Record<string, unknown> = {}, env: Record<string, string> = ENV, solutionId: string | null = null): Synth {
  const outdir = fs.mkdtempSync(path.join(os.tmpdir(), 'factoryos-cdk-'));
  const app = new App({ outdir, context: { ...BASE, ...context } });
  const { main, edge } = buildApp(app, env, solutionId);
  const assembly = app.synth();
  const tags: Record<string, Record<string, string>> = {};
  for (const s of assembly.stacks) tags[s.stackName] = s.tags;
  const json = assembly.getStackByName(main.stackName).template;
  const edgeJson = assembly.getStackByName(edge.stackName).template;
  return {
    main: Template.fromJSON(json),
    edge: Template.fromJSON(edgeJson),
    json,
    edgeJson,
    outdir,
    mainStackName: main.stackName,
    edgeStackName: edge.stackName,
    tags,
  };
}

const base = synth();
const resources = (s: Synth, type: string) => Object.entries<any>(s.json.Resources).filter(([, r]) => r.Type === type);
const instanceStatements = (s: Synth): any[] => resources(s, 'AWS::IAM::Policy')
  .find(([, r]) => r.Properties.Roles[0].Ref === PINNED_IDS.instanceRole)![1].Properties.PolicyDocument.Statement;
const list = (v: unknown): any[] => (Array.isArray(v) ? v : [v]);
/** A resource of a statement as text, `${AWS::Partition}` for the partition (`arn:${AWS::Partition}:…`). */
const arnText = (r: any): string => (typeof r === 'string' ? r : r['Fn::Join'] ? r['Fn::Join'][1].map((x: any) => (typeof x === 'string' ? x
  : x.Ref ? `\${${x.Ref}}` : `<${JSON.stringify(x)}>`)).join('') : `<${JSON.stringify(r)}>`);
const allPolicies = (s: Synth): any[] => [s.json, s.edgeJson].flatMap((t) => Object.values<any>(t.Resources))
  .filter((r) => r.Type === 'AWS::IAM::Policy' || r.Type === 'AWS::IAM::ManagedPolicy')
  .flatMap((r) => r.Properties.PolicyDocument.Statement);

describe('the contract', () => {
  it('names the stacks <stack> and <stack>-edge (us-east-1), both tagged', () => {
    expect(base.mainStackName).toBe('factoryos');
    expect(base.edgeStackName).toBe('factoryos-edge');
    expect(base.tags.factoryos[INSTALLER_FORMAT_TAG]).toBe('1');
    expect(base.tags['factoryos-edge'][INSTALLER_FORMAT_TAG]).toBe('1');
    const s = synth({ stackName: 'demo-1' });
    expect([s.mainStackName, s.edgeStackName]).toEqual(['demo-1', 'demo-1-edge']);
  });

  it('has exactly the outputs of deployment/CONTRACT.md', () => {
    expect(Object.keys(base.json.Outputs).sort()).toEqual(
      ['GateVerified', 'InstanceId', 'LogGroup', 'ReleaseDocument', 'SourceBucket', 'Url', 'UserPoolClientId', 'UserPoolDomain', 'UserPoolId'].sort(),
    );
    const o = base.json.Outputs;
    expect(o.Url.Value).toEqual({ 'Fn::Join': ['', ['https://', { 'Fn::GetAtt': ['Distribution', 'DomainName'] }, '/']] });
    expect(o.UserPoolId.Value).toEqual({ Ref: PINNED_IDS.userPool });
    expect(o.UserPoolClientId.Value).toEqual({ Ref: PINNED_IDS.userPoolClient });
    expect(o.InstanceId.Value).toEqual({ Ref: PINNED_IDS.instance });
    expect(o.SourceBucket.Value).toEqual({ Ref: PINNED_IDS.sourceBucket });
    expect(o.ReleaseDocument.Value).toEqual({ Ref: PINNED_IDS.releaseDocument });
    expect(o.LogGroup.Value).toEqual({ Ref: PINNED_IDS.logGroup });
    expect(JSON.stringify(o.UserPoolDomain.Value)).toContain('.auth.eu-central-1.amazoncognito.com');
    expect(o.GateVerified.Value).toEqual({ 'Fn::GetAtt': [PINNED_IDS.verify, 'Verified'] });
  });

  it('the SSM document <stack>-Release runs the inlined release-handoff.sh as root with a 600 s timeout', () => {
    const doc = base.json.Resources[PINNED_IDS.releaseDocument];
    expect(doc.Type).toBe('AWS::SSM::Document');
    expect(doc.Properties.Name).toBe('factoryos-Release');
    expect(doc.Properties.DocumentType).toBe('Command');
    expect(doc.Properties.UpdateMethod).toBe('NewVersion');
    const step = doc.Properties.Content.mainSteps[0];
    expect(step.action).toBe('aws:runShellScript');
    expect(step.inputs.timeoutSeconds).toBe('600');
    const script = JSON.stringify(step.inputs.runCommand);
    for (const line of readBootstrap('release-handoff.sh').trim().split('\n')) expect(script).toContain(JSON.stringify(line).slice(1, -1));
    expect(doc.Properties.Content.parameters).toBeUndefined();
  });

  it('the log group is /factoryos/<stack> with 30 days', () => {
    base.main.hasResourceProperties('AWS::Logs::LogGroup', { LogGroupName: '/factoryos/factoryos', RetentionInDays: 30 });
  });

  it('the source bucket: no public access, SSL only, SSE-S3, versioned, deleted with the stack (not auto-emptied)', () => {
    const b = base.json.Resources[PINNED_IDS.sourceBucket];
    expect(b.Type).toBe('AWS::S3::Bucket');
    expect(b.Properties.BucketName).toBeUndefined();
    expect(b.Properties.VersioningConfiguration).toEqual({ Status: 'Enabled' });
    expect(b.Properties.PublicAccessBlockConfiguration).toEqual({
      BlockPublicAcls: true, BlockPublicPolicy: true, IgnorePublicAcls: true, RestrictPublicBuckets: true,
    });
    expect(b.Properties.BucketEncryption.ServerSideEncryptionConfiguration[0].ServerSideEncryptionByDefault.SSEAlgorithm).toBe('AES256');
    expect(b.DeletionPolicy).toBe('Delete');
    const autoDelete = resources(base, 'Custom::S3AutoDeleteObjects').map(([, r]) => JSON.stringify(r.Properties.BucketName));
    expect(autoDelete.join()).not.toContain(PINNED_IDS.sourceBucket);
    const policy = resources(base, 'AWS::S3::BucketPolicy').find(([, r]) => r.Properties.Bucket.Ref === PINNED_IDS.sourceBucket)!;
    expect(JSON.stringify(policy[1])).toContain('aws:SecureTransport');
  });

  it('the access-log bucket: the sink of the source bucket and the distribution, not versioned, 90 days, auto-emptied', () => {
    // Not versioned: logs keep arriving while the stack is deleted, and a versioned bucket failed to delete (test
    // install 2611122014-dc89, DELETE_FAILED "You must delete all versions in the bucket").
    const src = base.json.Resources[PINNED_IDS.sourceBucket];
    const id: string = src.Properties.LoggingConfiguration.DestinationBucketName.Ref;
    expect(src.Properties.LoggingConfiguration.LogFilePrefix).toBe('source-bucket/');
    const [, dist] = resources(base, 'AWS::CloudFront::Distribution')[0];
    expect(dist.Properties.DistributionConfig.Logging.Bucket['Fn::GetAtt'][0]).toBe(id);
    const b = base.json.Resources[id];
    expect(b.Type).toBe('AWS::S3::Bucket');
    expect(b.Properties.BucketName).toBeUndefined();
    expect(b.Properties.VersioningConfiguration).toBeUndefined();
    expect(b.Properties.LifecycleConfiguration.Rules).toEqual([
      { AbortIncompleteMultipartUpload: { DaysAfterInitiation: 7 }, ExpirationInDays: 90, Status: 'Enabled' },
    ]);
    expect(b.Properties.PublicAccessBlockConfiguration).toEqual({
      BlockPublicAcls: true, BlockPublicPolicy: true, IgnorePublicAcls: true, RestrictPublicBuckets: true,
    });
    expect(b.Properties.BucketEncryption.ServerSideEncryptionConfiguration[0].ServerSideEncryptionByDefault.SSEAlgorithm).toBe('AES256');
    expect(b.DeletionPolicy).toBe('Delete');
    const autoDelete = resources(base, 'Custom::S3AutoDeleteObjects').map(([, r]) => r.Properties.BucketName);
    expect(autoDelete).toContainEqual({ Ref: id });
    // The source bucket is the only other bucket, and stays versioned (its exports are user data).
    expect(resources(base, 'AWS::S3::Bucket').map(([k]) => k).sort()).toEqual([id, PINNED_IDS.sourceBucket].sort());
  });

  it('the instance role has exactly the S3 / SSM permissions of the contract', () => {
    const statements = instanceStatements(base);
    expect(statements.map((s: any) => s.Sid)).toEqual([
      'SsmChannels', 'SsmAgent', 'SsmAgentAccount', 'AgentLogs', 'AgentMetrics', 'ReadReleases', 'ListReleases', 'ReleaseParameters', 'ReleaseState',
      'OriginSecret',
    ]);
    expect(JSON.stringify(statements)).not.toMatch(/bedrock|secretsmanager/);
    const text = JSON.stringify(statements);
    expect(text).toContain('/releases/*');
    expect(text).toContain('/vendor/*');
    expect(text).toContain(':parameter/factoryos/factoryos/release/wanted');
    expect(statements.find((s: any) => s.Sid === 'ReleaseState').Action).toBe('ssm:PutParameter');
    expect(base.json.Resources[PINNED_IDS.instanceRole].Properties.ManagedPolicyArns).toBeUndefined();
  });

  it('no AmazonSSMManagedInstanceCore or CloudWatchAgentServerPolicy (nor any managed policy) on the instance role', () => {
    for (const s of [base, synth({ bedrock: 'true', kiroSecretName: 'k', detailedMonitoring: 'true' })]) {
      const text = JSON.stringify(s.json) + JSON.stringify(s.edgeJson);
      expect(text).not.toContain('AmazonSSMManagedInstanceCore');
      expect(text).not.toContain('CloudWatchAgentServerPolicy');
      expect(s.json.Resources[PINNED_IDS.instanceRole].Properties.ManagedPolicyArns).toBeUndefined();
    }
  });

  it('the SSM agent: the channels and the agent actions, scoped where IAM allows; ssm:GetParameter* only on the release parameters and the origin secret', () => {
    const st = instanceStatements(base);
    const bySid = (sid: string) => st.find((x: any) => x.Sid === sid);
    expect(bySid('SsmChannels').Action.sort()).toEqual(['ec2messages:AcknowledgeMessage', 'ec2messages:DeleteMessage', 'ec2messages:FailMessage',
      'ec2messages:GetEndpoint', 'ec2messages:GetMessages', 'ec2messages:SendReply', 'ssmmessages:CreateControlChannel', 'ssmmessages:CreateDataChannel',
      'ssmmessages:OpenControlChannel', 'ssmmessages:OpenDataChannel']);
    expect(bySid('SsmChannels').Resource).toBe('*');
    // No wildcard action anywhere in either stack's policies.
    for (const x of allPolicies(base)) for (const a of list(x.Action)) expect(a, JSON.stringify(x)).not.toContain('*');
    expect(bySid('SsmAgent').Action.sort()).toEqual(['ssm:DescribeAssociation', 'ssm:DescribeDocument', 'ssm:GetDocument', 'ssm:ListInstanceAssociations',
      'ssm:PutComplianceItems', 'ssm:UpdateAssociationStatus', 'ssm:UpdateInstanceAssociationStatus', 'ssm:UpdateInstanceInformation']);
    expect(list(bySid('SsmAgent').Resource).map(arnText)).toEqual([
      'arn:${AWS::Partition}:ec2:eu-central-1:111122223333:instance/*',
      'arn:${AWS::Partition}:ssm:eu-central-1:111122223333:association/*',
      'arn:${AWS::Partition}:ssm:eu-central-1:111122223333:document/*',
      'arn:${AWS::Partition}:ssm:eu-central-1::document/*',
    ]);
    // Only the actions IAM gives no resource type.
    expect(bySid('SsmAgentAccount')).toMatchObject({
      Action: ['ssm:ListAssociations', 'ssm:PutInventory', 'ssm:GetManifest', 'ssm:GetDeployablePatchSnapshotForInstance'], Resource: '*',
    });
    // No ssm:GetParameter / GetParameters / GetParametersByPath on * (or any wildcard), in any policy of either stack.
    for (const s of [base, synth({ bedrock: 'true', kiroSecretName: 'k' }), synth({}, { ...ENV, CDK_DEFAULT_REGION: 'us-east-1' })]) {
      for (const x of allPolicies(s)) {
        const actions = list(x.Action);
        if (!actions.some((a: string) => /^ssm:(GetParameter|\*|Get\*)/.test(a) || a === '*')) continue;
        for (const r of list(x.Resource).map(arnText)) expect(r, JSON.stringify(x)).not.toMatch(/^\*$|:parameter\/\*|\*$/);
      }
    }
    const reads = st.filter((x: any) => list(x.Action).some((a: string) => a.startsWith('ssm:GetParameter')));
    expect(reads.map((x: any) => x.Sid)).toEqual(['ReleaseParameters', 'OriginSecret']);
    expect(list(reads[0].Resource).map(arnText)).toEqual([
      'arn:${AWS::Partition}:ssm:eu-central-1:111122223333:parameter/factoryos/factoryos/release/wanted',
      'arn:${AWS::Partition}:ssm:eu-central-1:111122223333:parameter/factoryos/factoryos/release/state',
    ]);
    expect(reads[1].Action).toBe('ssm:GetParameter');
    expect(list(reads[1].Resource).map(arnText)).toEqual(['arn:${AWS::Partition}:ssm:eu-central-1:111122223333:parameter/factoryos/factoryos/origin-secret']);
  });

  it('the CloudWatch agent: logs on the stack\'s log group only, metrics in its namespace only, no ec2:DescribeTags', () => {
    const st = instanceStatements(base);
    const logsSt = st.filter((x: any) => list(x.Action).some((a: string) => a.startsWith('logs:')));
    expect(logsSt).toHaveLength(1);
    expect(logsSt[0].Action).toEqual(['logs:CreateLogStream', 'logs:PutLogEvents', 'logs:DescribeLogStreams']);
    expect(list(logsSt[0].Resource).map(arnText)).toEqual([
      `arn:\${AWS::Partition}:logs:eu-central-1:111122223333:log-group:\${${PINNED_IDS.logGroup}}`,
      `<${JSON.stringify({ 'Fn::GetAtt': [PINNED_IDS.logGroup, 'Arn'] })}>`,
    ]);
    expect(base.json.Resources[PINNED_IDS.logGroup].Properties.LogGroupName).toBe('/factoryos/factoryos');
    const metrics = st.filter((x: any) => list(x.Action).some((a: string) => a.startsWith('cloudwatch:')));
    expect(metrics).toEqual([{
      Sid: 'AgentMetrics', Effect: 'Allow', Action: 'cloudwatch:PutMetricData', Resource: '*',
      Condition: { StringEquals: { 'cloudwatch:namespace': AGENT_METRICS_NAMESPACE } },
    }]);
    // user-data.sh's agent configuration collects logs only and appends no EC2 dimensions or tags.
    const ud = readBootstrap('user-data.sh');
    expect(ud).not.toMatch(/append_dimensions|ec2tagger|"namespace"/);
    expect(JSON.stringify(st)).not.toMatch(/ec2:DescribeTags|logs:PutRetentionPolicy|logs:CreateLogGroup/);
  });

  it('bedrock=true and kiroSecretName add only InvokeModel(+Stream) on the named profiles and models, and the one secret', () => {
    const s = synth({ bedrock: 'true', kiroSecretName: 'factoryos/kiro-api-key' });
    const st = instanceStatements(s);
    const actions = ['bedrock:InvokeModel', 'bedrock:InvokeModelWithResponseStream'];
    const profiles = [
      'arn:${AWS::Partition}:bedrock:eu-central-1:111122223333:inference-profile/eu.anthropic.claude-opus-5',
      'arn:${AWS::Partition}:bedrock:eu-central-1:111122223333:inference-profile/eu.anthropic.claude-sonnet-5',
    ];
    const bedrock = st.filter((x: any) => JSON.stringify(x.Action).includes('bedrock:'));
    expect(bedrock.map((x: any) => x.Sid)).toEqual(['BedrockInferenceProfiles', 'BedrockFoundationModels']);
    const [prof, fm] = bedrock;
    expect(prof.Action).toEqual(actions);
    expect(prof.Condition).toBeUndefined();
    expect(list(prof.Resource).map(arnText)).toEqual(profiles);
    expect(fm.Action).toEqual(actions);
    expect(list(fm.Resource).map(arnText)).toEqual([
      'arn:${AWS::Partition}:bedrock:*::foundation-model/anthropic.claude-opus-5',
      'arn:${AWS::Partition}:bedrock:*::foundation-model/anthropic.claude-sonnet-5',
    ]);
    expect(Object.keys(fm.Condition)).toEqual(['StringEquals']);
    expect(Object.keys(fm.Condition.StringEquals)).toEqual(['bedrock:InferenceProfileArn']);
    expect(list(fm.Condition.StringEquals['bedrock:InferenceProfileArn']).map(arnText)).toEqual(profiles);
    expect(JSON.stringify(bedrock)).not.toMatch(/foundation-model\/\*|inference-profile\/\*/);
    const kiro = st.find((x: any) => x.Sid === 'KiroApiKey');
    expect(kiro.Action).toBe('secretsmanager:GetSecretValue');
    expect(JSON.stringify(kiro.Resource)).toContain(':secret:factoryos/kiro-api-key-??????');
    // The release (not the user data) learns the secret's name.
    const run = JSON.stringify(s.json.Resources[PINNED_IDS.releaseDocument].Properties.Content.mainSteps[0].inputs.runCommand);
    expect(run).toContain("export FACTORYOS_KIRO_SECRET='factoryos/kiro-api-key'");  // pragma: allowlist secret (a secret's name, not its value)
    expect(run).toContain("export FACTORYOS_BEDROCK='1'");
  });

  it('bedrockModelIds names the profiles exactly (one foundation model per distinct model)', () => {
    const ids = 'global.anthropic.claude-haiku-4-5-20251001-v1:0,us.anthropic.claude-haiku-4-5-20251001-v1:0,eu.amazon.nova-pro-v1:0';
    const st = instanceStatements(synth({ bedrock: 'true', bedrockModelIds: ids }));
    const prof = st.find((x: any) => x.Sid === 'BedrockInferenceProfiles');
    const fm = st.find((x: any) => x.Sid === 'BedrockFoundationModels');
    expect(list(prof.Resource).map(arnText)).toEqual(ids.split(',').map((id) => `arn:\${AWS::Partition}:bedrock:eu-central-1:111122223333:inference-profile/${id}`));
    expect(list(fm.Resource).map(arnText)).toEqual([
      'arn:${AWS::Partition}:bedrock:*::foundation-model/anthropic.claude-haiku-4-5-20251001-v1:0',
      'arn:${AWS::Partition}:bedrock:*::foundation-model/amazon.nova-pro-v1:0',
    ]);
    expect(list(fm.Condition.StringEquals['bedrock:InferenceProfileArn']).map(arnText)).toEqual(list(prof.Resource).map(arnText));
    // Bedrock off: no Bedrock statement, whatever bedrockModelIds says.
    expect(JSON.stringify(instanceStatements(synth({ bedrockModelIds: ids })))).not.toContain('bedrock:');
  });

  it('the default profiles are the agentic planner\'s, with its Region prefix', () => {
    expect(defaultBedrockModelIds('eu-central-1')).toEqual(['eu.anthropic.claude-opus-5', 'eu.anthropic.claude-sonnet-5']);
    expect(defaultBedrockModelIds('us-east-1')).toEqual(['us.anthropic.claude-opus-5', 'us.anthropic.claude-sonnet-5']);
    expect([inferenceProfilePrefix('ap-northeast-1'), inferenceProfilePrefix('ca-central-1'), inferenceProfilePrefix('sa-east-1'),
      inferenceProfilePrefix('me-central-1')]).toEqual(['apac.', 'us.', 'us.', 'global.']);
    const cfg = readConfig({ tryGetContext: (k: string) => ({ ...BASE, bedrock: 'true' } as Record<string, unknown>)[k] }, { ...ENV, CDK_DEFAULT_REGION: 'us-west-2' });
    expect(cfg.bedrockModelIds).toEqual(['us.anthropic.claude-opus-5', 'us.anthropic.claude-sonnet-5']);
    const off = readConfig({ tryGetContext: (k: string) => (BASE as Record<string, unknown>)[k] }, ENV);
    expect(off.bedrockModelIds).toEqual([]);
  });
});

describe('the instance', () => {
  const instance = base.json.Resources[PINNED_IDS.instance];

  it('Ubuntu AMI from context, Graviton, 200 GB gp3 encrypted root, private subnet, no key pair, IMDSv2 hop 1', () => {
    const p = instance.Properties;
    expect(p.ImageId).toBe('ami-0123456789abcdef0');
    expect(p.InstanceType).toBe('m7g.2xlarge');
    expect(p.BlockDeviceMappings).toEqual([{ DeviceName: '/dev/sda1', Ebs: { DeleteOnTermination: true, Encrypted: true, VolumeSize: 200, VolumeType: 'gp3' } }]);
    expect(p.KeyName).toBeUndefined();
    expect(p.NetworkInterfaces[0].AssociatePublicIpAddress).toBe(false);
    expect(p.NetworkInterfaces[0].SubnetId.Ref).toMatch(/private/i);
    const lt = base.json.Resources[PINNED_IDS.launchTemplate];
    expect(lt.Properties.LaunchTemplateData).toEqual({
      MetadataOptions: { HttpEndpoint: 'enabled', HttpPutResponseHopLimit: 1, HttpTokens: 'required', InstanceMetadataTags: 'disabled' },
    });
    expect(p.LaunchTemplate.LaunchTemplateId).toEqual({ Ref: PINNED_IDS.launchTemplate });
  });

  it('user data: the stack values, then deployment/bootstrap/user-data.sh', () => {
    const joined = JSON.stringify(instance.Properties.UserData);
    for (const v of ["export FACTORYOS_STACK='factoryos'", "export AWS_REGION='eu-central-1'", "export AWS_DEFAULT_REGION='eu-central-1'",
      "export FACTORYOS_LOG_GROUP='/factoryos/factoryos'", "export FACTORYOS_PUBLIC='1'", "export FACTORYOS_SOURCE_BUCKET='"]) {
      expect(joined).toContain(v);
    }
    expect(joined).toContain(JSON.stringify({ Ref: PINNED_IDS.sourceBucket }));
    const body = readBootstrap('user-data.sh').split('\n').slice(1).join('\n');
    expect(joined).toContain(JSON.stringify(body).slice(1, -1));
    expect(joined.match(/#!\/bin\/bash/g)).toHaveLength(1);
  });

  it('a missing bootstrap file fails the synth with its name', () => {
    expect(() => readBootstrap('no-such-file.sh')).toThrow(/deployment\/bootstrap\/no-such-file.sh is missing/);
  });
});

describe('stability (an update never replaces the instance, its volume, the pool or the bucket)', () => {
  it('pins the logical IDs', () => {
    const want: Record<string, string> = {
      [PINNED_IDS.instance]: 'AWS::EC2::Instance',
      [PINNED_IDS.launchTemplate]: 'AWS::EC2::LaunchTemplate',
      [PINNED_IDS.instanceSecurityGroup]: 'AWS::EC2::SecurityGroup',
      [PINNED_IDS.instanceRole]: 'AWS::IAM::Role',
      [PINNED_IDS.logGroup]: 'AWS::Logs::LogGroup',
      [PINNED_IDS.originIngress]: 'AWS::EC2::SecurityGroupIngress',
      [PINNED_IDS.userPool]: 'AWS::Cognito::UserPool',
      [PINNED_IDS.userPoolClient]: 'AWS::Cognito::UserPoolClient',
      [PINNED_IDS.userPoolDomain]: 'AWS::Cognito::UserPoolDomain',
      [PINNED_IDS.sourceBucket]: 'AWS::S3::Bucket',
      [PINNED_IDS.distribution]: 'AWS::CloudFront::Distribution',
      [PINNED_IDS.releaseDocument]: 'AWS::SSM::Document',
      [PINNED_IDS.verify]: 'Custom::VerifyAnonymousDenial',
    };
    for (const [id, type] of Object.entries(want)) expect(base.json.Resources[id]?.Type, id).toBe(type);
    // The root volume is the instance's (BlockDeviceMappings): asserted above, unchanged below.
    expect(Object.values<any>(base.json.Resources).filter((r) => r.Type === 'AWS::EC2::Volume')).toHaveLength(0);
  });

  it('other contexts (a release, Bedrock, Kiro) leave the instance, its launch template, the pool and the bucket identical', () => {
    const other = synth({ bedrock: 'true', bedrockModelIds: 'global.anthropic.claude-haiku-4-5', kiroSecretName: 'factoryos/kiro',  // pragma: allowlist secret (a secret's name, not its value)
      release: 'v9.9.9', version: 'dev-0123456789ab', unrelated: 'x' });
    for (const id of [PINNED_IDS.instance, PINNED_IDS.launchTemplate, PINNED_IDS.userPool, PINNED_IDS.sourceBucket,
      PINNED_IDS.instanceSecurityGroup, PINNED_IDS.instanceRole, PINNED_IDS.logGroup]) {
      expect(other.json.Resources[id], id).toEqual(base.json.Resources[id]);
    }
    // The user data (an update must never change it) does not depend on any option.
    expect(other.json.Resources[PINNED_IDS.instance].Properties.UserData).toEqual(base.json.Resources[PINNED_IDS.instance].Properties.UserData);
  });

  it('the intended inputs change only their own instance property', () => {
    const other = synth({ instanceType: 'm8g.4xlarge', amiId: 'ami-1234567890abcdef0', volumeGb: '300', detailedMonitoring: 'true' });
    const a = base.json.Resources[PINNED_IDS.instance].Properties;
    const b = other.json.Resources[PINNED_IDS.instance].Properties;
    const changed = Object.keys({ ...a, ...b }).filter((k) => JSON.stringify(a[k]) !== JSON.stringify(b[k])).sort();
    expect(changed).toEqual(['BlockDeviceMappings', 'ImageId', 'InstanceType', 'Monitoring']);
  });

  it('no CloudFormation users in the pool', () => {
    expect(resources(base, 'AWS::Cognito::UserPoolUser')).toHaveLength(0);
    expect(resources(base, 'AWS::Cognito::UserPoolUserToGroupAttachment')).toHaveLength(0);
  });
});

describe('the gate and the deploy order', () => {
  it('the user pool: native only, no self sign-up, TOTP MFA required, strong passwords, invitation with the URL', () => {
    const p = base.json.Resources[PINNED_IDS.userPool].Properties;
    expect(p.AdminCreateUserConfig.AllowAdminCreateUserOnly).toBe(true);
    expect(p.MfaConfiguration).toBe('ON');
    expect(p.EnabledMfas).toEqual(['SOFTWARE_TOKEN_MFA']);
    expect(p.Policies.PasswordPolicy).toMatchObject({ MinimumLength: 14, RequireLowercase: true, RequireUppercase: true, RequireNumbers: true, RequireSymbols: true });
    const invite = JSON.stringify(p.AdminCreateUserConfig.InviteMessageTemplate.EmailMessage);
    expect(invite).toContain('Distribution');
    expect(invite).toContain('{username}');
    expect(invite).toContain('{####}');
    expect(resources(base, 'AWS::Cognito::UserPoolIdentityProvider')).toHaveLength(0);
    const client = base.json.Resources[PINNED_IDS.userPoolClient].Properties;
    expect(client.GenerateSecret).toBe(false);
    expect(client.AllowedOAuthFlows).toEqual(['code']);
    expect(client.AllowedOAuthScopes).toEqual(['openid', 'email', 'profile']);
    expect(client.SupportedIdentityProviders).toEqual(['COGNITO']);
    expect(JSON.stringify(client.CallbackURLs)).toContain('/_auth/callback');
    base.main.resourceCountIs('AWS::Cognito::ManagedLoginBranding', 1);
    expect(base.json.Resources[PINNED_IDS.userPoolDomain].Properties.ManagedLoginVersion).toBe(2);
  });

  it('the hosted domain prefix is derived from stack, account and region, without reserved words', () => {
    expect(base.json.Resources[PINNED_IDS.userPoolDomain].Properties.Domain).toBe(domainPrefix('factoryos', '111122223333', 'eu-central-1'));
    expect(domainPrefix('my-aws-cognito', '111122223333', 'eu-central-1')).toMatch(/^my-x-x-[0-9a-f]{10}$/);
    expect(domainPrefix('a', '111122223333', 'eu-central-1')).not.toBe(domainPrefix('a', '444455556666', 'eu-central-1'));
  });

  it('the distribution: VPC origin to the instance on port 80, HTTPS viewers, no caching, all viewer headers but Host', () => {
    const d = base.json.Resources[PINNED_IDS.distribution].Properties.DistributionConfig;
    const vo = resources(base, 'AWS::CloudFront::VpcOrigin');
    expect(vo).toHaveLength(1);
    expect(vo[0][1].Properties.VpcOriginEndpointConfig).toMatchObject({ HTTPPort: 80, OriginProtocolPolicy: 'http-only' });
    expect(JSON.stringify(vo[0][1].Properties.VpcOriginEndpointConfig.Arn)).toContain(`"Ref":"${PINNED_IDS.instance}"`);
    expect(d.Origins[0].VpcOriginConfig.VpcOriginId).toEqual({ 'Fn::GetAtt': [vo[0][0], 'Id'] });
    const b = d.DefaultCacheBehavior;
    expect(b.ViewerProtocolPolicy).toBe('https-only');
    expect(b.CachePolicyId).toBe('4135ea2d-6df8-44a3-9df3-4b5a84be39ad'); // CachingDisabled
    expect(b.OriginRequestPolicyId).toBe('b689b0a8-53d0-40ab-baf2-68738e2966ac'); // AllViewerExceptHostHeader
    expect(b.ResponseHeadersPolicyId).toBe('67f7725c-6f97-4210-82d7-5512b31e9d03'); // SecurityHeadersPolicy
    expect(b.AllowedMethods).toContain('POST');
    expect(b.LambdaFunctionAssociations).toHaveLength(1);
    expect(b.LambdaFunctionAssociations[0].EventType).toBe('viewer-request');
    expect(d.PriceClass).toBe('PriceClass_100');
    expect(d.Logging.IncludeCookies).toBe(false);
  });

  it('the instance security group starts with no ingress; the only rule waits for VerifyAnonymousDenial', () => {
    const sg = base.json.Resources[PINNED_IDS.instanceSecurityGroup];
    expect(sg.Properties.SecurityGroupIngress).toBeUndefined();
    const ingress = resources(base, 'AWS::EC2::SecurityGroupIngress').filter(([, r]) => JSON.stringify(r.Properties.GroupId).includes(PINNED_IDS.instanceSecurityGroup));
    expect(ingress).toHaveLength(1);
    const [id, rule] = ingress[0];
    expect(id).toBe(PINNED_IDS.originIngress);
    expect(rule.Properties).toMatchObject({ IpProtocol: 'tcp', FromPort: 80, ToPort: 80 });
    expect(rule.Properties.CidrIp).toBeUndefined();
    expect(rule.DependsOn).toContain(PINNED_IDS.verify);
    const lookupId = rule.Properties.SourceSecurityGroupId['Fn::GetAtt'][0];
    const lookup = base.json.Resources[lookupId];
    expect(lookup.DependsOn).toEqual(expect.arrayContaining([PINNED_IDS.verify, PINNED_IDS.distribution]));
    expect(lookup.Type).toBe('Custom::VpcOriginSecurityGroup');
    expect(lookup.Properties.GroupName).toBe('CloudFront-VPCOrigins-Service-SG');
    expect(rule.Properties.SourceSecurityGroupId).toEqual({ 'Fn::GetAtt': [lookupId, 'GroupId'] });
    // Nothing else opens the instance: no other ingress anywhere in the stack.
    expect(resources(base, 'AWS::EC2::SecurityGroupIngress')).toHaveLength(1);
  });

  it('VerifyAnonymousDenial re-runs on every gate change: edge version and config hash in its properties', () => {
    const v = base.json.Resources[PINNED_IDS.verify];
    expect(Object.keys(v.Properties)).toEqual(expect.arrayContaining(['EdgeVersionArn', 'ConfigHash', 'DistributionDomain', 'ClientId', 'CognitoDomain', 'Issuer']));
    expect(v.Properties.ConfigHash).toMatch(/^[0-9a-f]{64}$/);
    expect(v.DependsOn).toEqual(expect.arrayContaining(['Distribution']));
    expect(v.DependsOn.some((d: string) => d.startsWith('EdgeConfig'))).toBe(true);
    const other = synth({ stackName: 'other' });
    expect(other.json.Resources[PINNED_IDS.verify].Properties.ConfigHash).not.toBe(v.Properties.ConfigHash);
  });

  it('the edge configuration is written to SSM in us-east-1 under the literal name the edge code is bundled with', () => {
    const [, writer] = resources(base, 'Custom::AWS').find(([id]) => id.startsWith('EdgeConfig'))!;
    const create = JSON.stringify(writer.Properties.Create);
    expect(create).toContain('\\"region\\":\\"us-east-1\\"');
    expect(create).toContain(edgeConfigParameterName('factoryos'));
    const settings = fs.readdirSync(base.outdir)
      .filter((d) => d.startsWith('asset.') && fs.existsSync(path.join(base.outdir, d, 'settings.json')))
      .map((d) => JSON.parse(fs.readFileSync(path.join(base.outdir, d, 'settings.json'), 'utf8')));
    expect(settings).toEqual([{
      parameterName: '/factoryos/factoryos/edge-config', secretParameterName: '/factoryos/factoryos/edge-origin-secret', parameterRegion: 'us-east-1',
    }]);
  });

  it('the edge configuration: Create never overwrites (another Region\'s install fails the deploy), Update does, Delete only its own', () => {
    const [, writer] = resources(base, 'Custom::AWS').find(([id]) => id.startsWith('EdgeConfig'))!;
    // The calls are JSON strings with tokens (Fn::Join); the literal parts carry what is checked here.
    const call = (k: string) => JSON.stringify(writer.Properties[k]).replace(/\\"/g, '"');
    expect(call('Create')).toContain('"Overwrite":false');
    expect(call('Create')).not.toContain('"Overwrite":true');
    expect(call('Update')).toContain('"Overwrite":true');
    // The value names the Region that wrote it: the stack's (eu-central-1), not only the parameter's (us-east-1).
    for (const k of ['Create', 'Update']) {
      expect(call(k)).toContain('"action":"putParameter"');
      expect(call(k)).toMatch(/\\+"region\\+":\\+"eu-central-1\\+"/);
      expect(call(k)).toContain('"physicalResourceId":{"id":"/factoryos/factoryos/edge-config"}');
    }
    // Delete: the parameter its physical id names (a failed create's is not the other installation's parameter).
    expect(call('Delete')).toContain('"Name":"PHYSICAL:RESOURCEID:"');
    expect(call('Delete')).toContain(`"ignoreErrorCodesMatching":"${EDGE_CONFIG_DELETE_IGNORED.replace(/\\/g, '\\\\')}"`);
    expect(new RegExp(EDGE_CONFIG_DELETE_IGNORED).test('ParameterNotFound')).toBe(true);
    expect(new RegExp(EDGE_CONFIG_DELETE_IGNORED).test('ThrottlingException')).toBe(false);
    const policy = resources(base, 'AWS::IAM::Policy').find(([id]) => id.startsWith('EdgeConfig'))![1];
    expect(JSON.stringify(policy.Properties.PolicyDocument.Statement)).toContain(':ssm:us-east-1:111122223333:parameter/factoryos/factoryos/edge-config');
  });

  it('the edge function (us-east-1): viewer limits, may read only its parameter', () => {
    const fns = Object.values<any>(base.edgeJson.Resources).filter((r) => r.Type === 'AWS::Lambda::Function');
    expect(fns).toHaveLength(1);
    expect(fns[0].Properties).toMatchObject({ Handler: 'index.handler', MemorySize: 128, Timeout: 5 });
    expect(fns[0].Properties.Environment).toBeUndefined();
    const policy = Object.values<any>(base.edgeJson.Resources).find((r) => r.Type === 'AWS::IAM::Policy');
    expect(JSON.stringify(policy.Properties.PolicyDocument.Statement)).toContain(':ssm:us-east-1:111122223333:parameter/factoryos/factoryos/edge-config');
  });

  it('the edge function code knows no internal sign-in', () => {
    const dirs = [EDGE_SOURCE_DIR, ...fs.readdirSync(base.outdir).filter((d) => d.startsWith('asset.')).map((d) => path.join(base.outdir, d))];
    const files = dirs.flatMap((d) => fs.readdirSync(d).filter((f) => /\.(mjs|json)$/.test(f)).map((f) => path.join(d, f)));
    const edgeFiles = files.filter((f) => fs.readFileSync(f, 'utf8').includes('__Host-fos_id') || f.endsWith('settings.json'));
    expect(edgeFiles.length).toBeGreaterThan(1);
    for (const f of edgeFiles) {
      const text = fs.readFileSync(f, 'utf8').toLowerCase();
      expect(text, f).not.toContain(['mid', 'way'].join(''));
      expect(text, f).not.toContain('federat');
    }
  });
});

describe('context validation and variants', () => {
  it('rejects bad context with a message naming the key', () => {
    const bad: Array<[Record<string, unknown>, RegExp]> = [
      [{ amiId: '' }, /amiId is required/],
      [{ amiId: 'ami-xyz' }, /amiId/],
      [{ stackName: 'Factory' }, /stackName/],
      [{ stackName: 'a'.repeat(32) }, /stackName/],
      [{ stackName: 'aws-test' }, /must not start with aws/],
      [{ instanceType: 'm7i.2xlarge' }, /Graviton/],
      [{ instanceType: 'm7g' }, /Graviton/],
      [{ volumeGb: '50' }, /volumeGb/],
      [{ bedrock: 'yes' }, /bedrock/],
      [{ kiroSecretName: 'arn:aws:secretsmanager:x' }, /kiroSecretName/],  // pragma: allowlist secret (a secret's ARN, not its value)
      [{ bedrock: 'true', bedrockModelIds: 'anthropic.claude-opus-5' }, /bedrockModelIds/],
      [{ bedrock: 'true', bedrockModelIds: 'eu.anthropic.*' }, /bedrockModelIds/],
      [{ bedrock: 'true', bedrockModelIds: '*' }, /bedrockModelIds/],
      [{ bedrock: 'true', bedrockModelIds: 'arn:aws:bedrock:eu-central-1:111122223333:inference-profile/eu.anthropic.claude-opus-5' }, /bedrockModelIds/],
      [{ bedrock: 'true', bedrockModelIds: 'eu.anthropic.claude-opus-5,,eu.anthropic.claude-sonnet-5' }, /bedrockModelIds/],
      [{ bedrock: 'true', bedrockModelIds: 'eu.anthropic.claude-opus-5,eu.anthropic.claude-opus-5' }, /bedrockModelIds lists an id twice/],
      [{ bedrock: 'true', bedrockModelIds: Array.from({ length: 21 }, (_, i) => `eu.anthropic.m${i}`).join(',') }, /more than 20/],
      [{ bedrock: 'true', bedrockModelIds: 'eu.anthropic.claude opus' }, /bedrockModelIds/],
      [{ bedrockModelIds: 'EU.Anthropic.Claude' }, /bedrockModelIds/],
    ];
    for (const [ctx, msg] of bad) {
      const app = new App({ outdir: fs.mkdtempSync(path.join(os.tmpdir(), 'factoryos-cdk-')), context: { ...BASE, ...ctx } });
      expect(() => buildApp(app, ENV), JSON.stringify(ctx)).toThrow(msg);
    }
    const app = new App({ context: BASE });
    expect(() => buildApp(app, { CDK_DEFAULT_REGION: 'eu-central-1' })).toThrow(/CDK_DEFAULT_ACCOUNT/);
  });

  it('accepts Graviton families', () => {
    for (const t of ['m7g.2xlarge', 'm7gd.4xlarge', 'c7gn.xlarge', 'r8g.large', 't4g.xlarge', 'm6g.metal']) {
      expect(() => synth({ instanceType: t }), t).not.toThrow();
    }
  });

  it('synthesizes (cdk-nag clean) with every option on, and in us-east-1 with the same two stacks', () => {
    expect(() => synth({ bedrock: true, kiroSecretName: 'kiro', detailedMonitoring: true, volumeGb: 500 })).not.toThrow();  // pragma: allowlist secret (a secret's name, not its value)
    const east = synth({}, { ...ENV, CDK_DEFAULT_REGION: 'us-east-1' });
    expect([east.mainStackName, east.edgeStackName]).toEqual(['factoryos', 'factoryos-edge']);
    expect(Object.values<any>(east.edgeJson.Resources).filter((r) => r.Type === 'AWS::Lambda::Function')).toHaveLength(1);
    expect(Object.values<any>(east.json.Resources).filter((r) => r.Type === 'AWS::Lambda::Function' && JSON.stringify(r).includes('viewer-request gate'))).toHaveLength(0);
    expect(JSON.stringify(east.json.Parameters)).toContain('/factoryos/factoryos/edge-auth-version');
  });

  it('SOLUTION_ID: null or SO####; once set, both stack descriptions end with (SO####)', () => {
    // Null until Alchemy assigns one; then lib/app.ts and public-export/guidance.json's solutionId name the same id
    // (scripts/tests/test_export_public.py checks that they agree).
    expect(SOLUTION_ID === null || SOLUTION_ID_FORMAT.test(SOLUTION_ID)).toBe(true);
    const real = synth({}, ENV, SOLUTION_ID);
    for (const t of [real.json, real.edgeJson]) {
      if (SOLUTION_ID === null) expect(t.Description).not.toMatch(/\(SO/);
      else expect(t.Description.endsWith(` (${SOLUTION_ID})`)).toBe(true);
    }
    expect(base.json.Description).toBe('FactoryOS (Physical AI Production Designer): public deployment');
    expect(base.json.Description).not.toMatch(/\(SO/);
    expect(stackDescription('x', null)).toBe('x');
    expect(stackDescription('FactoryOS', 'SO9123')).toBe('FactoryOS (SO9123)');
    for (const bad of ['SO12', 'so9123', 'SO91234', '(SO9123)', '']) expect(() => stackDescription('x', bad), bad).toThrow(/SOLUTION_ID/);
    const s = synth({}, ENV, 'SO9123');
    expect(s.json.Description).toBe('FactoryOS (Physical AI Production Designer): public deployment (SO9123)');
    expect(s.edgeJson.Description).toBe('FactoryOS factoryos: the CloudFront viewer-request gate (Lambda@Edge) (SO9123)');
    const east = synth({}, { ...ENV, CDK_DEFAULT_REGION: 'us-east-1' }, 'SO9123');
    for (const t of [east.json, east.edgeJson]) expect(t.Description).toMatch(/ \(SO9123\)$/);
  });

  it('an unacknowledged cdk-nag finding fails the synth', () => {
    const outdir = fs.mkdtempSync(path.join(os.tmpdir(), 'factoryos-cdk-'));
    const app = new App({ outdir, context: BASE });
    const { main } = buildApp(app, ENV);
    // A second, unjustified managed policy on the instance role.
    const role = main.node.findChild('InstanceRole') as iam.Role;
    role.addManagedPolicy(iam.ManagedPolicy.fromAwsManagedPolicyName('ReadOnlyAccess'));
    expect(() => app.synth()).toThrow(/AwsSolutions-IAM4\[Policy::arn:<AWS::Partition>:iam::aws:policy\/ReadOnlyAccess\]/);
  });

  it('the managed policies the role used to have are no longer acknowledged (adding one back fails the synth)', () => {
    for (const name of ['AmazonSSMManagedInstanceCore', 'CloudWatchAgentServerPolicy']) {
      const app = new App({ outdir: fs.mkdtempSync(path.join(os.tmpdir(), 'factoryos-cdk-')), context: BASE });
      const { main } = buildApp(app, ENV);
      (main.node.findChild('InstanceRole') as iam.Role).addManagedPolicy(iam.ManagedPolicy.fromAwsManagedPolicyName(name));
      expect(() => app.synth(), name).toThrow(new RegExp(`AwsSolutions-IAM4\\[Policy::arn:<AWS::Partition>:iam::aws:policy/${name}\\]`));
    }
  });

  it('a Bedrock wildcard is not acknowledged (any model or any profile fails the synth)', () => {
    for (const resource of ['arn:${AWS::Partition}:bedrock:*::foundation-model/*', 'arn:${AWS::Partition}:bedrock:*:111122223333:inference-profile/*']) {
      const app = new App({ outdir: fs.mkdtempSync(path.join(os.tmpdir(), 'factoryos-cdk-')), context: { ...BASE, bedrock: 'true' } });
      const { main } = buildApp(app, ENV);
      (main.node.findChild('InstanceRole') as iam.Role).addToPolicy(new iam.PolicyStatement({
        actions: ['bedrock:InvokeModel'], resources: [resource.replace('${AWS::Partition}', main.partition)],
      }));
      expect(() => app.synth(), resource).toThrow(/AwsSolutions-IAM5\[Resource::arn:<AWS::Partition>:bedrock:\*:/);
    }
  });
});

describe('VPC Block Public Access', () => {
  it('excludes only the instance subnet (private, no internet gateway route), before the VPC origin', () => {
    const ex = resources(base, 'AWS::EC2::VPCBlockPublicAccessExclusion');
    expect(ex).toHaveLength(1);
    const [exId, exRes] = ex[0];
    expect(exRes.Properties.InternetGatewayExclusionMode).toBe('allow-bidirectional');
    expect(exRes.Properties.VpcId).toBeUndefined();
    const instance = base.json.Resources[PINNED_IDS.instance];
    expect(exRes.Properties.SubnetId).toEqual(instance.Properties.NetworkInterfaces[0].SubnetId);
    expect(exRes.Properties.SubnetId.Ref).toBeDefined();
    // The excluded subnet's route table has no route to the internet gateway (only the NAT gateway's).
    const subnetId = exRes.Properties.SubnetId.Ref;
    const assoc = resources(base, 'AWS::EC2::SubnetRouteTableAssociation').find(([, r]) => r.Properties.SubnetId.Ref === subnetId)!;
    const routes = resources(base, 'AWS::EC2::Route').filter(([, r]) => r.Properties.RouteTableId.Ref === assoc[1].Properties.RouteTableId.Ref);
    expect(routes.length).toBeGreaterThan(0);
    for (const [, r] of routes) {
      expect(r.Properties.GatewayId).toBeUndefined();
      expect(r.Properties.NatGatewayId).toBeDefined();
    }
    const vpcOrigins = resources(base, 'AWS::CloudFront::VpcOrigin');
    expect(vpcOrigins).toHaveLength(1);
    expect(list(vpcOrigins[0][1].DependsOn)).toContain(exId);
  });
});
