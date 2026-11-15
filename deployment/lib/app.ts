import { App, Stack, Tags, Validations } from 'aws-cdk-lib';
import { AwsSolutionsChecks } from 'cdk-nag';
import { DeploymentConfig, readConfig } from './config';
import { EDGE_REGION } from './edge';
import { FactoryosStack } from './factoryos-stack';
import { acknowledgeEdge } from './nag';

export const INSTALLER_FORMAT_TAG = 'factoryos:installer-format';

/**
 * The AWS Solutions / Guidance tracking id (`SO` and digits) once one is assigned, or null. When set, both stacks'
 * descriptions end with ` (SO####)`, which is how CloudFormation deployments of the Guidance are counted.
 */
export const SOLUTION_ID: string | null = null;
export const SOLUTION_ID_FORMAT = /^SO\d{4}$/; // scripts/guidance_check.py S6

/** A stack description, with ` (<solution id>)` appended when there is one. */
export function stackDescription(text: string, solutionId: string | null = SOLUTION_ID): string {
  if (solutionId === null) return text;
  if (!SOLUTION_ID_FORMAT.test(solutionId)) throw new Error(`SOLUTION_ID must match ${SOLUTION_ID_FORMAT.source}, not ${JSON.stringify(solutionId)}`);
  const described = `${text} (${solutionId})`;
  // CloudFormation's limit for a stack description.
  if (described.length > 1024) throw new Error('the stack description is longer than 1024 characters');
  return described;
}

export interface FactoryosApp {
  readonly config: DeploymentConfig;
  readonly main: FactoryosStack;
  readonly edge: Stack;
}

/**
 * The two stacks of deployment/CONTRACT.md: `<stack>` in CDK_DEFAULT_REGION (everything but the edge function) and
 * `<stack>-edge` in us-east-1 (the Lambda@Edge function). cdk-nag's AWS Solutions rules run on both; a finding that
 * is not acknowledged (with its reason, lib/nag.ts) fails the synth.
 */
export function buildApp(app: App, env: NodeJS.ProcessEnv = process.env, solutionId: string | null = SOLUTION_ID): FactoryosApp {
  const config = readConfig(app.node, env);
  const edge = new Stack(app, `${config.stackName}-edge`, {
    stackName: `${config.stackName}-edge`,
    env: { account: config.account, region: EDGE_REGION },
    description: stackDescription(`FactoryOS ${config.stackName}: the CloudFront viewer-request gate (Lambda@Edge)`, solutionId),
  });
  const main = new FactoryosStack(app, config.stackName, {
    stackName: config.stackName,
    env: { account: config.account, region: config.region },
    description: stackDescription('FactoryOS (Physical AI Production Designer): public deployment', solutionId),
    config,
    edgeStack: edge,
  });
  acknowledgeEdge(edge);
  Tags.of(app).add(INSTALLER_FORMAT_TAG, '1');
  Validations.of(app).addPlugins(new AwsSolutionsChecks(app));
  return { config, main, edge };
}
