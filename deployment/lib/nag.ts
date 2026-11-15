import { Stack, Validations } from 'aws-cdk-lib';
import { IConstruct } from 'constructs';
import { DeploymentConfig, foundationModelId } from './config';

/**
 * The cdk-nag (AWS Solutions) findings this app accepts, each with its reason. Anything else fails the synth
 * (lib/app.ts adds the rules; test/stacks.test.ts synthesizes every variant).
 */
const BASIC_EXECUTION = 'AwsSolutions::AwsSolutions-IAM4[Policy::arn:<AWS::Partition>:iam::aws:policy/service-role/AWSLambdaBasicExecutionRole]';

export function ack(scope: IConstruct, id: string, reason: string): void {
  Validations.of(scope).acknowledge({ id, reason });
}

export function acknowledgeEdge(edge: Stack): void {
  ack(edge, BASIC_EXECUTION, 'Lambda@Edge writes its logs to CloudWatch Logs in every region it runs in; the managed policy grants only that.');
  ack(edge, 'AwsSolutions::AwsSolutions-L1',
    'Lambda@Edge supports fewer runtimes than Lambda: nodejs22.x is the newest one it supports.');
}

export interface MainParts {
  readonly cfg: DeploymentConfig;
  readonly instance: IConstruct;
  readonly role: IConstruct;
  readonly distribution: IConstruct;
  readonly pool: IConstruct;
  readonly originSg: IConstruct;
}

export function acknowledgeMain(stack: Stack, p: MainParts): void {
  ack(stack, BASIC_EXECUTION,
    'The stack\'s custom resource Lambdas (gate check, edge-config writer, origin-secret writer, security group lookup, and CDK\'s '
    + 'S3 auto-delete and cross-Region ARN reader) need it for CloudWatch Logs only; their other permissions are statements of their own roles.');

  // The SSM agent and the CloudWatch agent (instanceAgentStatements in lib/factoryos-stack.ts; no managed policies).
  ack(p.role, 'AwsSolutions::AwsSolutions-IAM5[Resource::*]',
    'Only actions without resource-level permissions: the Session Manager and Run Command channels (the ssmmessages and '
    + 'ec2messages actions, named: the only way into the instance, no SSH, no public IP), the SSM agent\'s '
    + 'ListAssociations, PutInventory, GetManifest and GetDeployablePatchSnapshotForInstance, and cloudwatch:PutMetricData '
    + 'limited to the CloudWatch agent\'s namespace by a condition. No ssm:GetParameter* on *.');
  const r = p.cfg.region;
  const a = p.cfg.account;
  ack(p.role, `AwsSolutions::AwsSolutions-IAM5[Resource::arn:<AWS::Partition>:ec2:${r}:${a}:instance/*]`,
    'The SSM agent\'s registration, association status and compliance calls on its own instance: the instance\'s ARN '
    + 'cannot be named in its own role\'s policy (a dependency cycle), so this account\'s instances in this Region.');
  ack(p.role, `AwsSolutions::AwsSolutions-IAM5[Resource::arn:<AWS::Partition>:ssm:${r}:${a}:association/*]`,
    'The SSM agent reads and reports the State Manager associations that target its instance (which ones is the account\'s choice), in this Region.');
  ack(p.role, `AwsSolutions::AwsSolutions-IAM5[Resource::arn:<AWS::Partition>:ssm:${r}:${a}:document/*]`,
    'The SSM agent reads the documents of the associations and commands sent to it (this Region, read only).');
  ack(p.role, `AwsSolutions::AwsSolutions-IAM5[Resource::arn:<AWS::Partition>:ssm:${r}::document/*]`,
    'The SSM agent reads the AWS-owned documents (AWS-RunShellScript, AWS-RunPatchBaseline, …) it is sent (this Region, read only).');
  ack(p.role, 'AwsSolutions::AwsSolutions-IAM5[Resource::<SourceBucket.Arn>/releases/*]',
    'The instance reads the exports the installer copies to releases/ of the stack\'s own source bucket (read only).');
  ack(p.role, 'AwsSolutions::AwsSolutions-IAM5[Resource::<SourceBucket.Arn>/vendor/*]',
    'The instance reads the optional umarell tarball under vendor/ of the stack\'s own source bucket (read only).');
  // The Kiro secret's statement (secret:<name>-??????) needs no acknowledgement: IAM5 flags only '*'.
  // Bedrock (opt-in, bedrock=true): the named inference profiles carry no wildcard; their foundation models do in
  // the Region only (a cross-Region profile routes to the model in several Regions), behind bedrock:InferenceProfileArn.
  for (const model of new Set(p.cfg.bedrockModelIds.map(foundationModelId))) {
    ack(p.role, `AwsSolutions::AwsSolutions-IAM5[Resource::arn:<AWS::Partition>:bedrock:*::foundation-model/${model}]`,
      `InvokeModel(+WithResponseStream) on ${model} only through this account's inference profiles of context bedrockModelIds `
      + '(condition bedrock:InferenceProfileArn): a cross-Region profile routes to the model in any of its destination Regions.');
  }

  if (!p.cfg.detailedMonitoring) {
    ack(p.instance, 'AwsSolutions::AwsSolutions-EC28',
      'Detailed monitoring is optional (context detailedMonitoring=true; extra cost); the CloudWatch agent ships logs and metrics.');
  }
  ack(p.instance, 'AwsSolutions::AwsSolutions-EC29',
    'A single instance by design (its volume holds the designs). The installer refuses any update that would replace it, and the uninstall must be able to delete it.');
  ack(p.instance, 'CloudFormation-Validate::W9010',
    'The AMI comes from the installer (context amiId): the newest Ubuntu 24.04 arm64 on install, the running one on update, so a newer AMI never replaces the instance.');

  ack(p.distribution, 'AwsSolutions::AwsSolutions-CFR4',
    'The distribution uses its default *.cloudfront.net certificate (no custom domain), whose security policy cannot be raised; viewers are HTTPS only.');
  ack(p.distribution, 'AwsSolutions::AwsSolutions-CFR1',
    'No geo restriction: the users are whoever the deployer adds to the user pool, wherever they are.');
  ack(p.distribution, 'AwsSolutions::AwsSolutions-CFR2',
    'Every request is authenticated at the edge (Cognito session, Lambda@Edge) before it reaches the origin; WAF is left to the deployer (cost).');

  ack(p.pool, 'AwsSolutions::AwsSolutions-COG8',
    'Essentials feature plan (managed login): users are created by the deployer only (no self sign-up) and need TOTP MFA; the Plus plan\'s threat protection is an opt-in cost.');

  // The VpcOriginSecurityGroupLookup function's own role (no other custom resource uses it).
  ack(p.originSg, 'AwsSolutions::AwsSolutions-IAM5[Resource::*]',
    'ec2:DescribeSecurityGroups supports no resource-level permissions (finds the VPC origins\' security group of this VPC); '
    + 'the role of a function of its own, with no other permission.');
}
