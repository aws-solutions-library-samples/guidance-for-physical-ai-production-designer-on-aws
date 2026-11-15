import * as fs from 'node:fs';
import * as path from 'node:path';
import { AssetHashType, Aws, DockerImage, Duration, RemovalPolicy, Stack } from 'aws-cdk-lib';
import * as cloudfront from 'aws-cdk-lib/aws-cloudfront';
import * as iam from 'aws-cdk-lib/aws-iam';
import * as lambda from 'aws-cdk-lib/aws-lambda';
import * as logs from 'aws-cdk-lib/aws-logs';
import * as ssm from 'aws-cdk-lib/aws-ssm';

export const EDGE_REGION = 'us-east-1';
export const EDGE_SOURCE_DIR = path.join(__dirname, '..', 'lambda', 'edge-auth');

/** The SSM parameter (us-east-1) the edge function reads its configuration from; written by the main stack. */
export function edgeConfigParameterName(stackName: string): string {
  return `/factoryos/${stackName}/edge-config`;
}

/**
 * The SecureString (us-east-1) with the origin secret the edge function sends as `x-origin-verify` (deployment/CONTRACT.md
 * "Origin secret"): the edge side's copy, written by the main stack's OriginSecret custom resource, never by
 * CloudFormation (no template, no parameter of it carries the value), and by the installer when it rotates the secret.
 * A parameter of its own rather than a key of the edge configuration: that one is written by the EdgeConfig custom
 * resource from the template's tokens, and stays a String. Lambda@Edge reads it with ssm:GetParameter,
 * WithDecryption (the AWS managed key aws/ssm), next to the configuration.
 */
export function edgeOriginSecretParameterName(stackName: string): string {
  return `/factoryos/${stackName}/edge-origin-secret`;
}

/** settings.json, bundled next to the edge function's index.mjs (Lambda@Edge has no environment variables). */
export function edgeSettings(stackName: string): string {
  return `${JSON.stringify({
    parameterName: edgeConfigParameterName(stackName),
    secretParameterName: edgeOriginSecretParameterName(stackName),
    parameterRegion: EDGE_REGION,
  })}\n`;
}

/** The edge function's code: index.mjs plus the stack's settings.json (hashed by output: per stack name). */
export function edgeCode(stackName: string): lambda.Code {
  return lambda.Code.fromAsset(EDGE_SOURCE_DIR, {
    assetHashType: AssetHashType.OUTPUT,
    bundling: {
      // Never pulled: the local bundling below always succeeds (two files, no build).
      image: DockerImage.fromRegistry('scratch'),
      local: {
        tryBundle(outputDir: string): boolean {
          fs.copyFileSync(path.join(EDGE_SOURCE_DIR, 'index.mjs'), path.join(outputDir, 'index.mjs'));
          fs.writeFileSync(path.join(outputDir, 'settings.json'), edgeSettings(stackName));
          return true;
        },
      },
    },
  });
}

export interface EdgeAuth {
  /** The version CloudFront associates (viewer-request). */
  readonly version: lambda.IVersion;
  /** Its ARN (resolved in the main stack). */
  readonly versionArn: string;
}

/**
 * The viewer-request function, `cloudfront.experimental.EdgeFunction`, in the stack `<stack>-edge` (us-east-1).
 *
 * Main stack outside us-east-1: EdgeFunction itself puts the function into `edgeStack` (found by its `stackId`) and
 * reads the version's ARN back through its SSM parameter (refreshed whenever the version changes).
 * Main stack in us-east-1: EdgeFunction would put the function into the main stack; it is created in `edgeStack`
 * instead (the same stacks in every region), and the main stack reads the version's ARN from an SSM parameter
 * resolved on every deployment (an export would block every update of the function).
 */
export function createEdgeAuth(main: Stack, edgeStack: Stack, stackName: string): EdgeAuth {
  // The function's log group: a custom one (LoggingConfig), so Lambda@Edge uses this one name in every Region it runs
  // the function in, instead of /aws/lambda/us-east-1.<function> (CloudFront Developer Guide, "Edge function logs":
  // "If you specify a custom log group name, ... Lambda@Edge won't add the Region prefix. The log group name remains
  // the same across all other Regions where the function is invoked."). This stack creates it in us-east-1 only, with
  // the retention of the stack's other log groups; in the other Regions Lambda@Edge creates it on the first
  // invocation there, without a retention (CloudFormation cannot create resources in Regions it does not deploy to;
  // the uninstall is to delete them by this group's name, read from this stack's resources before deleting it).
  // The name is CloudFormation's (unique per stack instance), so a reinstall never meets a group that a previous
  // installation's replicas left behind.
  const logGroup = new logs.LogGroup(edgeStack, 'EdgeAuthLogs', {
    retention: logs.RetentionDays.ONE_MONTH,
    removalPolicy: RemovalPolicy.DESTROY,
  });
  const props: cloudfront.experimental.EdgeFunctionProps = {
    runtime: lambda.Runtime.NODEJS_22_X,
    handler: 'index.handler',
    code: edgeCode(stackName),
    memorySize: 128,
    timeout: Duration.seconds(5),
    description: `FactoryOS ${stackName}: CloudFront viewer-request gate (Cognito session)`,
    logGroup,
  };
  // Its configuration and the origin secret's edge copy, nothing else (the SecureString is decrypted with aws/ssm,
  // whose key policy admits this account's principals through SSM: no kms statement).
  const parameter = (name: string) => `arn:${Aws.PARTITION}:ssm:${EDGE_REGION}:${main.account}:parameter${name}`;
  const policy = new iam.PolicyStatement({
    actions: ['ssm:GetParameter'],
    resources: [parameter(edgeConfigParameterName(stackName)), parameter(edgeOriginSecretParameterName(stackName))],
  });

  if (main.region !== EDGE_REGION) {
    const fn = new cloudfront.experimental.EdgeFunction(main, 'EdgeAuth', { ...props, stackId: edgeStack.node.id });
    fn.addToRolePolicy(policy);
    return { version: fn, versionArn: fn.edgeArn };
  }

  const fn = new cloudfront.experimental.EdgeFunction(edgeStack, 'EdgeAuth', props);
  fn.addToRolePolicy(policy);
  const parameterName = `/factoryos/${stackName}/edge-auth-version`;
  new ssm.StringParameter(edgeStack, 'EdgeAuthVersionArn', { parameterName, stringValue: fn.edgeArn });
  main.addStackDependency(edgeStack, 'the main stack reads the edge function version ARN from its SSM parameter');
  const versionArn = ssm.StringParameter.valueForStringParameter(main, parameterName);
  return { version: lambda.Version.fromVersionArn(main, 'EdgeAuthVersion', versionArn), versionArn };
}
