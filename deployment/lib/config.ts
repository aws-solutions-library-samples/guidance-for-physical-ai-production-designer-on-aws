/**
 * The CDK context the installer passes (deployment/CONTRACT.md "Stacks"), validated: a bad value fails the synth with
 * a message naming the key, before anything is deployed.
 */
export interface DeploymentConfig {
  /** `<stack>`: the main stack's name; `<stack>-edge` is the edge function's stack in us-east-1. */
  readonly stackName: string;
  readonly account: string;
  readonly region: string;
  /** Ubuntu 24.04 arm64 AMI (the installer resolves it; on update the running instance's own). */
  readonly amiId: string;
  /** A Graviton type (the release is built natively for arm64). */
  readonly instanceType: string;
  /** The root volume (GiB). Changing it replaces the instance (and loses its data). */
  readonly volumeGb: number;
  /** Whether the instance role may invoke Bedrock models (the agentic planner). */
  readonly bedrock: boolean;
  /**
   * The cross-Region inference profiles (ids) the instance role may invoke when `bedrock` is on: context
   * `bedrockModelIds` (comma-separated), by default the profiles the agentic planner uses in this Region
   * (defaultBedrockModelIds). Empty when `bedrock` is off.
   */
  readonly bedrockModelIds: readonly string[];
  /** A Secrets Manager secret (name) with the Kiro API key, or ''. */
  readonly kiroSecretName: string;
  /** Detailed (1-minute) CloudWatch monitoring of the instance (extra cost). */
  readonly detailedMonitoring: boolean;
}

export const STACK_NAME = /^[a-z][a-z0-9-]{0,30}$/;
export const AMI_ID = /^ami-[0-9a-f]{8,17}$/;
/** Graviton families: m7g, m7gd, c7gn, r8g, t4g, x2gd, … (a `g` after the generation digit). */
export const GRAVITON_TYPE = /^[a-z]+\d+g[a-z]*\.(?:medium|large|xlarge|\d+xlarge|metal(?:-\d+xl)?)$/;
const SECRET_NAME = /^[A-Za-z0-9/_+=.@-]{1,512}$/;
/**
 * A system-defined cross-Region inference profile id: `<geography>.<provider>.<model>`, such as
 * the Region's geography prefix (`eu.`, `us.`, `apac.`, …, or `global.`) before a foundation model id of PLANNER_MODELS'
 * form (no ARN, no wildcard).
 */
export const INFERENCE_PROFILE_ID = /^(?:us|eu|apac|global|jp|au|ca|us-gov)\.[a-z0-9]+\.[a-z0-9][a-z0-9.-]*(?::\d+)?$/;
export const MAX_BEDROCK_MODELS = 20; // the installer's --bedrock-models limit

/**
 * The agentic planner's (umarell's agents/llm_model.py) models: its default (Opus 5) and its trajectory-planning
 * default (Sonnet 5), as the inference profiles it calls in this Region: it derives the profile's prefix from the
 * Region it calls Bedrock in, which is the stack's (deploy/provision.sh agenticRegion), by the same table.
 */
export const PLANNER_MODELS = ['anthropic.claude-opus-5', 'anthropic.claude-sonnet-5'] as const;

export function inferenceProfilePrefix(region: string): string {
  for (const [start, prefix] of [['eu-', 'eu.'], ['us-', 'us.'], ['ap-', 'apac.'], ['ca-', 'us.'], ['sa-', 'us.']]) {
    if (region.startsWith(start)) return prefix;
  }
  return 'global.';
}

export function defaultBedrockModelIds(region: string): string[] {
  return PLANNER_MODELS.map((m) => `${inferenceProfilePrefix(region)}${m}`);
}

/** The foundation model an inference profile id routes to: the id without its geography prefix. */
export function foundationModelId(profileId: string): string {
  return profileId.slice(profileId.indexOf('.') + 1);
}

function bedrockModelIds(v: unknown, region: string): string[] {
  const raw = str(v, '');
  if (!raw) return defaultBedrockModelIds(region);
  const ids = raw.split(',').map((x) => x.trim());
  const bad = ids.filter((id) => !INFERENCE_PROFILE_ID.test(id));
  if (bad.length) {
    throw new Error(`context bedrockModelIds must be comma-separated inference profile ids such as ${defaultBedrockModelIds(region)[0]} `
      + `(no ARNs, no wildcards), not ${JSON.stringify(bad.join(','))}`);
  }
  if (new Set(ids).size !== ids.length) throw new Error(`context bedrockModelIds lists an id twice: ${JSON.stringify(raw)}`);
  if (ids.length > MAX_BEDROCK_MODELS) throw new Error(`context bedrockModelIds lists more than ${MAX_BEDROCK_MODELS} ids`);
  return ids;
}

export interface ContextReader {
  tryGetContext(key: string): unknown;
}

function bool(key: string, v: unknown, dflt: boolean): boolean {
  if (v === undefined || v === null || v === '') return dflt;
  if (v === true || v === 'true') return true;
  if (v === false || v === 'false') return false;
  throw new Error(`context ${key} must be true or false, not ${JSON.stringify(v)}`);
}

function str(v: unknown, dflt: string): string {
  if (v === undefined || v === null) return dflt;
  return String(v).trim();
}

export function readConfig(ctx: ContextReader, env: NodeJS.ProcessEnv = process.env): DeploymentConfig {
  const stackName = str(ctx.tryGetContext('stackName'), 'factoryos');
  if (!STACK_NAME.test(stackName)) {
    throw new Error(`context stackName must match ${STACK_NAME.source}, not ${JSON.stringify(stackName)}`);
  }
  // AWS reserves these prefixes for SSM document names (`<stack>-Release`).
  if (/^(aws|amazon|amzn)/.test(stackName)) {
    throw new Error(`context stackName must not start with aws, amazon or amzn: ${stackName}`);
  }

  const account = str(env.CDK_DEFAULT_ACCOUNT, '');
  const region = str(env.CDK_DEFAULT_REGION, '');
  if (!/^\d{12}$/.test(account)) throw new Error('CDK_DEFAULT_ACCOUNT is not set (run through the installer, or set it for a synth)');
  if (!/^[a-z]{2}(-[a-z]+)+-\d$/.test(region)) throw new Error('CDK_DEFAULT_REGION is not set (run through the installer, or set it for a synth)');

  const amiId = str(ctx.tryGetContext('amiId'), '');
  if (!AMI_ID.test(amiId)) {
    throw new Error(`context amiId is required and must match ${AMI_ID.source} (the installer resolves the Ubuntu 24.04 arm64 AMI), not ${JSON.stringify(amiId)}`);
  }

  const instanceType = str(ctx.tryGetContext('instanceType'), 'm7g.2xlarge');
  if (!GRAVITON_TYPE.test(instanceType)) {
    throw new Error(`context instanceType must be a Graviton (arm64) type such as m7g.2xlarge, not ${JSON.stringify(instanceType)}`);
  }

  const volumeRaw = str(ctx.tryGetContext('volumeGb'), '200');
  const volumeGb = Number(volumeRaw);
  if (!/^\d+$/.test(volumeRaw) || volumeGb < 100 || volumeGb > 16384) {
    throw new Error(`context volumeGb must be an integer from 100 to 16384, not ${JSON.stringify(volumeRaw)}`);
  }

  const kiroSecretName = str(ctx.tryGetContext('kiroSecretName'), '');
  if (kiroSecretName && (!SECRET_NAME.test(kiroSecretName) || kiroSecretName.startsWith('arn:'))) {
    throw new Error(`context kiroSecretName must be a Secrets Manager secret name (not an ARN), not ${JSON.stringify(kiroSecretName)}`);
  }

  const bedrock = bool('bedrock', ctx.tryGetContext('bedrock'), false);
  // Validated even with bedrock off (a typo fails now, not when Bedrock is turned on).
  const models = bedrockModelIds(ctx.tryGetContext('bedrockModelIds'), region);

  return {
    stackName,
    account,
    region,
    amiId,
    instanceType,
    volumeGb,
    bedrock,
    bedrockModelIds: bedrock ? models : [],
    kiroSecretName,
    detailedMonitoring: bool('detailedMonitoring', ctx.tryGetContext('detailedMonitoring'), false),
  };
}
