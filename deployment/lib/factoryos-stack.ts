import * as crypto from 'node:crypto';
import * as fs from 'node:fs';
import * as path from 'node:path';
import {
  Aws, CfnOutput, CfnResource, CustomResource, Duration, Fn, RemovalPolicy, Stack, StackProps, Token,
} from 'aws-cdk-lib';
import * as cloudfront from 'aws-cdk-lib/aws-cloudfront';
import * as origins from 'aws-cdk-lib/aws-cloudfront-origins';
import * as cognito from 'aws-cdk-lib/aws-cognito';
import * as ec2 from 'aws-cdk-lib/aws-ec2';
import * as iam from 'aws-cdk-lib/aws-iam';
import * as lambda from 'aws-cdk-lib/aws-lambda';
import * as logs from 'aws-cdk-lib/aws-logs';
import * as s3 from 'aws-cdk-lib/aws-s3';
import * as ssm from 'aws-cdk-lib/aws-ssm';
import * as cr from 'aws-cdk-lib/custom-resources';
import { Construct } from 'constructs';
import { DeploymentConfig, foundationModelId } from './config';
import {
  createEdgeAuth, EDGE_REGION, EDGE_SOURCE_DIR, edgeConfigParameterName, edgeOriginSecretParameterName, edgeSettings,
} from './edge';
import { acknowledgeMain } from './nag';

export const BOOTSTRAP_DIR = path.join(__dirname, '..', 'bootstrap');
const VERIFY_SOURCE_DIR = path.join(__dirname, '..', 'lambda', 'verify-denial');
/** The stack's own custom resource Lambdas (OriginSecret, VpcOriginSecurityGroupLookup): one asset, one handler each. */
export const CUSTOM_RESOURCES_DIR = path.join(__dirname, '..', 'lambda', 'custom-resources');

/** The instance's copy of the origin secret (SecureString, the stack's Region; the edge's copy: edgeOriginSecretParameterName). */
export function originSecretParameterName(stackName: string): string {
  return `/factoryos/${stackName}/origin-secret`;
}
/**
 * The flow log role's logical id: the one CDK gave the role it created itself, kept so that an update changes that
 * role in place. A new role would replace the flow log (DeliverLogsPermissionArn requires replacement): CloudFormation
 * would create the new flow log while the old one, of the same VPC, traffic type and log group, still exists, which
 * EC2 may refuse as a duplicate.
 */
export const FLOW_LOG_ROLE_ID = 'VpcallIAMRole2920156A';
/**
 * Cognito's grace period for a rotated-out refresh token (RefreshTokenRotation.RetryGracePeriodSeconds, 0-60 s). The
 * gate refreshes in the viewer request, and a page whose tokens ran out sends several requests at once, each with the
 * same refresh token: without a grace period only the first would succeed and the others would sign the user out.
 */
export const REFRESH_ROTATION_GRACE = Duration.seconds(60);

/** Logical IDs pinned so a refactor cannot rename (replace) them; installer/infra.sh refuses only instance/volume/bucket/user-pool replacement. */
export const PINNED_IDS = {
  instance: 'Instance',
  launchTemplate: 'InstanceLaunchTemplate',
  instanceSecurityGroup: 'InstanceSecurityGroup',
  instanceRole: 'InstanceRole',
  userPool: 'UserPool',
  userPoolClient: 'UserPoolWebClient',
  userPoolDomain: 'UserPoolDomain',
  sourceBucket: 'SourceBucket',
  distribution: 'Distribution',
  logGroup: 'LogGroup',
  releaseDocument: 'ReleaseDocument',
  verify: 'VerifyAnonymousDenial',
  originIngress: 'OriginIngress',
} as const;

/** The security group CloudFront creates in a VPC for its VPC origins (the only source of the instance's ingress). */
export const VPC_ORIGINS_SECURITY_GROUP = 'CloudFront-VPCOrigins-Service-SG';
const SESSION_REFRESH_TTL = Duration.hours(12);
/**
 * The errors EdgeConfig's delete ignores: the parameter is gone, or the physical id is not a parameter of this stack's
 * (the log stream name of a failed create: an invalid name, or one its policy does not cover).
 */
export const EDGE_CONFIG_DELETE_IGNORED = '^(ParameterNotFound|ValidationException|AccessDeniedException)$';
/** The CloudWatch agent's metric namespace (its default; user-data.sh's agent configuration names no other). */
export const AGENT_METRICS_NAMESPACE = 'CWAgent';
const HANDOFF_DELIMITER = 'FACTORYOS_RELEASE_HANDOFF_EOF';

/** A file of deployment/bootstrap/ (the instance side's), inlined at synth: missing is an error, not an empty script. */
export function readBootstrap(name: string): string {
  const file = path.join(BOOTSTRAP_DIR, name);
  if (!fs.existsSync(file)) {
    throw new Error(`deployment/bootstrap/${name} is missing: the CDK app inlines it (deployment/CONTRACT.md)`);
  }
  const text = fs.readFileSync(file, 'utf8');
  if (!text.trim()) throw new Error(`deployment/bootstrap/${name} is empty`);
  return text;
}

function withoutShebang(script: string): string {
  return script.startsWith('#!') ? script.slice(script.indexOf('\n') + 1) : script;
}

/** Shell `export NAME='value'` (the values are validated names, ids or CloudFormation tokens of such). */
function exportLine(name: string, value: string): string {
  if (!Token.isUnresolved(value) && !/^[A-Za-z0-9_./:@+=-]*$/.test(value)) throw new Error(`unsafe value for ${name}`);
  return `export ${name}='${value}'`;
}

/** Cognito hosted domain prefix: unique per stack, account and region, without the words Cognito reserves. */
export function domainPrefix(stackName: string, account: string, region: string): string {
  const hash = crypto.createHash('sha256').update(`${stackName}/${account}/${region}`).digest('hex').slice(0, 10);
  const name = stackName.replace(/aws|amazon|cognito/g, 'x').replace(/-+$/, '');
  return `${name}-${hash}`;
}

export interface FactoryosStackProps extends StackProps {
  readonly config: DeploymentConfig;
  /** `<stack>-edge` in us-east-1 (the edge function's stack). */
  readonly edgeStack: Stack;
}

export class FactoryosStack extends Stack {
  constructor(scope: Construct, id: string, props: FactoryosStackProps) {
    super(scope, id, props);
    const cfg = props.config;
    const name = cfg.stackName;

    // ----- Logs, buckets ------------------------------------------------------------------------------------
    const logGroupName = `/factoryos/${name}`;
    const logGroup = new logs.LogGroup(this, 'FactoryosLogGroup', {
      logGroupName,
      retention: logs.RetentionDays.ONE_MONTH,
      removalPolicy: RemovalPolicy.DESTROY,
    });
    pin(logGroup, PINNED_IDS.logGroup);

    // Access logs of the distribution and the source bucket (not user data: emptied by the stack on delete). Not
    // versioned, on purpose (checkov CKV_AWS_21 accepted in public-export/scan/ash.yaml): the logs are append-only, so
    // versions recover nothing, and they keep arriving while the stack is deleted (CloudFront and S3 deliver late),
    // which left a versioned bucket with versions the delete could not keep up with (DELETE_FAILED "You must delete
    // all versions in the bucket", test install 2611122014-dc89). installer/uninstall.sh empties and retries a
    // bucket of the stack that still fails.
    const accessLogs = new s3.Bucket(this, 'AccessLogBucket', {
      blockPublicAccess: s3.BlockPublicAccess.BLOCK_ALL,
      enforceSSL: true,
      encryption: s3.BucketEncryption.S3_MANAGED,
      versioned: false,
      // CloudFront's standard logging writes with an ACL.
      objectOwnership: s3.ObjectOwnership.BUCKET_OWNER_PREFERRED,
      lifecycleRules: [{ expiration: Duration.days(90), abortIncompleteMultipartUploadAfter: Duration.days(7) }],
      removalPolicy: RemovalPolicy.DESTROY,
      autoDeleteObjects: true,
    });

    // The exports (releases/<version>.zip) and the optional umarell tarball (vendor/); the installer copies them.
    const sourceBucket = new s3.Bucket(this, 'Source', {
      blockPublicAccess: s3.BlockPublicAccess.BLOCK_ALL,
      enforceSSL: true,
      encryption: s3.BucketEncryption.S3_MANAGED,
      versioned: true,
      objectOwnership: s3.ObjectOwnership.BUCKET_OWNER_ENFORCED,
      serverAccessLogsBucket: accessLogs,
      serverAccessLogsPrefix: 'source-bucket/',
      lifecycleRules: [{ noncurrentVersionExpiration: Duration.days(30), abortIncompleteMultipartUploadAfter: Duration.days(7) }],
      // The uninstall empties it first (deployment/CONTRACT.md "Source bucket").
      removalPolicy: RemovalPolicy.DESTROY,
      autoDeleteObjects: false,
    });
    pin(sourceBucket, PINNED_IDS.sourceBucket);

    // ----- Network ------------------------------------------------------------------------------------------
    const flowLogs = new logs.LogGroup(this, 'VpcFlowLogs', {
      retention: logs.RetentionDays.ONE_MONTH,
      removalPolicy: RemovalPolicy.DESTROY,
    });
    // Assumed by VPC Flow Logs for this account's flow logs in this Region only (confused-deputy protection,
    // https://docs.aws.amazon.com/vpc/latest/userguide/flow-logs-iam-role.html: "The source account is the owner of
    // the flow log and the source ARN is the flow log ARN. If you don't know the flow log ID, you can replace that
    // portion of the ARN with a wildcard (*)": the flow log's id is not known before the flow log, which needs the
    // role). Its permissions: the flow-log destination's, which CDK adds (CreateLogStream, PutLogEvents,
    // DescribeLogStreams on this log group only).
    const flowLogRole = new iam.Role(this, 'FlowLogRole', {
      assumedBy: new iam.ServicePrincipal('vpc-flow-logs.amazonaws.com', {
        conditions: {
          StringEquals: { 'aws:SourceAccount': this.account },
          ArnLike: { 'aws:SourceArn': `arn:${Aws.PARTITION}:ec2:${this.region}:${this.account}:vpc-flow-log/*` },
        },
      }),
      description: `FactoryOS ${name}: VPC Flow Logs delivery to the stack's flow-log group`,
    });
    (flowLogRole.node.defaultChild as iam.CfnRole).overrideLogicalId(FLOW_LOG_ROLE_ID);
    const vpc = new ec2.Vpc(this, 'Vpc', {
      maxAzs: 2,
      natGateways: 1,
      subnetConfiguration: [
        { name: 'public', subnetType: ec2.SubnetType.PUBLIC, cidrMask: 24 },
        { name: 'private', subnetType: ec2.SubnetType.PRIVATE_WITH_EGRESS, cidrMask: 24 },
      ],
      flowLogs: {
        all: { destination: ec2.FlowLogDestination.toCloudWatchLogs(flowLogs, flowLogRole), trafficType: ec2.FlowLogTrafficType.ALL },
      },
    });

    // ----- The instance -------------------------------------------------------------------------------------
    // No ingress here: the only rule, port 80 from CloudFront's VPC origins, is added once the gate is verified
    // (OriginIngress below). Outbound: apt, crates, git, S3, SSM.
    const securityGroup = new ec2.SecurityGroup(this, 'InstanceSecurityGroup', {
      vpc,
      allowAllOutbound: true,
      description: `FactoryOS ${name}: the instance (ingress only from CloudFront VPC origins, after the gate check)`,
    });
    pin(securityGroup, PINNED_IDS.instanceSecurityGroup);

    // No managed policies (AmazonSSMManagedInstanceCore would let every terminal user of the instance read every
    // Parameter Store value of the account, CloudWatchAgentServerPolicy write to and re-configure any log group):
    // the statements of instanceAgentStatements, each as narrow as IAM allows.
    const role = new iam.Role(this, 'InstanceRole', {
      assumedBy: new iam.ServicePrincipal('ec2.amazonaws.com'),
      description: `FactoryOS ${name}: the instance`,
    });
    pin(role, PINNED_IDS.instanceRole);
    for (const st of instanceAgentStatements(this, logGroup)) role.addToPolicy(st);
    const objects = (prefix: string) => `${sourceBucket.bucketArn}/${prefix}/*`;
    role.addToPolicy(new iam.PolicyStatement({
      sid: 'ReadReleases',
      actions: ['s3:GetObject'],
      resources: [objects('releases'), objects('vendor')],
    }));
    role.addToPolicy(new iam.PolicyStatement({
      sid: 'ListReleases',
      actions: ['s3:ListBucket'],
      resources: [sourceBucket.bucketArn],
      conditions: { StringLike: { 's3:prefix': ['releases/*', 'vendor/*'] } },
    }));
    const releaseParameter = (leaf: string) => `arn:${Aws.PARTITION}:ssm:${this.region}:${this.account}:parameter/factoryos/${name}/release/${leaf}`;
    role.addToPolicy(new iam.PolicyStatement({
      sid: 'ReleaseParameters',
      actions: ['ssm:GetParameter'],
      resources: [releaseParameter('wanted'), releaseParameter('state')],
    }));
    role.addToPolicy(new iam.PolicyStatement({
      sid: 'ReleaseState',
      actions: ['ssm:PutParameter'],
      resources: [releaseParameter('state')],
    }));
    // The instance's copy of the origin secret (the release writes it into the backend's EnvironmentFile).
    role.addToPolicy(new iam.PolicyStatement({
      sid: 'OriginSecret',
      actions: ['ssm:GetParameter'],
      resources: [`arn:${Aws.PARTITION}:ssm:${this.region}:${this.account}:parameter${originSecretParameterName(name)}`],
    }));
    if (cfg.kiroSecretName) {
      role.addToPolicy(new iam.PolicyStatement({
        sid: 'KiroApiKey',
        actions: ['secretsmanager:GetSecretValue'],
        // Secrets Manager appends a 6-character suffix to a secret's ARN.
        resources: [`arn:${Aws.PARTITION}:secretsmanager:${this.region}:${this.account}:secret:${cfg.kiroSecretName}-??????`],
      }));
    }
    if (cfg.bedrock) {
      for (const st of bedrockStatements(this, cfg.bedrockModelIds)) role.addToPolicy(st);
    }

    // User data: the stack's values, then deployment/bootstrap/user-data.sh. Nothing in it depends on the release,
    // the instance type or the optional settings, so an update never changes it (a change would stop and start the
    // instance; cloud-init would not run it again anyway).
    const stackEnv: Array<[string, string]> = [
      ['FACTORYOS_STACK', name],
      ['AWS_REGION', this.region],
      ['AWS_DEFAULT_REGION', this.region],
      ['FACTORYOS_SOURCE_BUCKET', sourceBucket.bucketName],
      ['FACTORYOS_LOG_GROUP', logGroupName],
      ['FACTORYOS_PUBLIC', '1'],
    ];
    const userDataScript = withoutShebang(readBootstrap('user-data.sh'));
    const userData = ec2.UserData.custom([
      '#!/bin/bash',
      '# FactoryOS public deployment: the stack\'s values (deployment/CONTRACT.md "The instance"), then user-data.sh.',
      ...stackEnv.map(([k, v]) => exportLine(k, v)),
      userDataScript,
    ].join('\n'));
    if (Buffer.byteLength(userDataScript) > 15 * 1024) {
      throw new Error('deployment/bootstrap/user-data.sh is too large for EC2 user data (16 KiB with the prelude)');
    }

    // IMDSv2 only, hop limit 1 (no container on the host reaches the role). Never changed: a new launch template
    // version would replace the instance.
    const launchTemplate = new ec2.CfnLaunchTemplate(this, 'InstanceLaunchTemplate', {
      launchTemplateData: {
        metadataOptions: { httpEndpoint: 'enabled', httpTokens: 'required', httpPutResponseHopLimit: 1, instanceMetadataTags: 'disabled' },
      },
    });
    launchTemplate.overrideLogicalId(PINNED_IDS.launchTemplate);

    const instance = new ec2.Instance(this, 'Instance', {
      vpc,
      vpcSubnets: { subnetType: ec2.SubnetType.PRIVATE_WITH_EGRESS },
      instanceType: new ec2.InstanceType(cfg.instanceType),
      machineImage: ec2.MachineImage.genericLinux({ [this.region]: cfg.amiId }),
      securityGroup,
      role,
      userData,
      userDataCausesReplacement: false,
      associatePublicIpAddress: false,
      detailedMonitoring: cfg.detailedMonitoring,
      ssmSessionPermissions: false,
      propagateTagsToVolumeOnCreation: true,
      blockDevices: [{
        deviceName: '/dev/sda1',
        volume: ec2.BlockDeviceVolume.ebs(cfg.volumeGb, {
          volumeType: ec2.EbsDeviceVolumeType.GP3,
          encrypted: true,
          deleteOnTermination: true,
        }),
      }],
    });
    const cfnInstance = instance.node.defaultChild as ec2.CfnInstance;
    cfnInstance.overrideLogicalId(PINNED_IDS.instance);
    // The first-boot setup needs the internet at once (apt, the AWS CLI): the instance waits for its subnets' NAT
    // routes (CloudFormation could start it first). DependsOn only: an existing instance is not touched. Even with the
    // routes in place a new instance can have no path to the mirrors for a minute or two (test installs rf-old2,
    // rf-old3): user-data.sh retries every network step (FACTORYOS_NET_WAIT).
    for (const subnet of vpc.selectSubnets({ subnetType: ec2.SubnetType.PRIVATE_WITH_EGRESS }).subnets) {
      cfnInstance.node.addDependency(subnet.internetConnectivityEstablished);
    }
    cfnInstance.launchTemplate = { launchTemplateId: launchTemplate.ref, version: launchTemplate.attrLatestVersionNumber };

    // ----- The SSM document that starts a release -----------------------------------------------------------
    const handoff = readBootstrap('release-handoff.sh');
    if (handoff.split('\n').some((l) => l.trim() === HANDOFF_DELIMITER)) {
      throw new Error(`deployment/bootstrap/release-handoff.sh must not contain the line ${HANDOFF_DELIMITER}`);
    }
    // The release's view of the stack: the same values as stack.env plus the settings an update may change (they
    // never go into the user data). The handoff keeps /etc/factoryos/stack.env current with them.
    const releaseEnv: Array<[string, string]> = [
      ...stackEnv,
      ['FACTORYOS_KIRO_SECRET', cfg.kiroSecretName],
      ['FACTORYOS_BEDROCK', cfg.bedrock ? '1' : '0'],
      // The name, never the value: the release reads the SecureString with the instance role.
      ['FACTORYOS_ORIGIN_SECRET_PARAMETER', originSecretParameterName(name)],
    ];
    const documentName = `${name}-Release`;
    const releaseDocument = new ssm.CfnDocument(this, 'ReleaseCommandDocument', {
      name: documentName,
      documentType: 'Command',
      documentFormat: 'JSON',
      // A changed handoff becomes a new (default) version of the same document, never a replacement.
      updateMethod: 'NewVersion',
      content: {
        schemaVersion: '2.2',
        description: `FactoryOS ${name}: starts the release in /factoryos/${name}/release/wanted (returns at once; the build runs in factoryos-release.service).`,
        mainSteps: [{
          action: 'aws:runShellScript',
          name: 'releaseHandoff',
          inputs: {
            timeoutSeconds: '600',
            runCommand: [
              'set -eu',
              ...releaseEnv.map(([k, v]) => exportLine(k, v)),
              // For /run/factoryos and the script only: release-handoff.sh sets umask 022 itself (what it creates,
              // /opt/factoryos the backend's user must traverse, must not be 0700).
              'umask 077',
              'install -d -m 0700 /run/factoryos',
              `cat > /run/factoryos/release-handoff.sh <<'${HANDOFF_DELIMITER}'`,
              ...handoff.replace(/\n$/, '').split('\n'),
              HANDOFF_DELIMITER,
              'exec /bin/bash /run/factoryos/release-handoff.sh',
            ],
          },
        }],
      },
    });
    releaseDocument.overrideLogicalId(PINNED_IDS.releaseDocument);

    // ----- The origin secret (deployment/CONTRACT.md "Origin secret") -----------------------------------------
    // Generated by the OriginSecret custom resource when the stack (or, on an update of an older installation, the
    // resource) is created (lambda/custom-resources/origin-secret.mjs): the template carries the parameters' names
    // only, never the value. CloudFormation cannot create a SecureString itself (AWS::SSM::Parameter has no
    // SecureString type). An update never changes the secret; rotation is the installer's (--rotate-origin-secret,
    // deployment/CONTRACT.md). The custom resource's role may write and delete those two parameters only; the edge
    // role may read the edge copy (lib/edge.ts), the instance role the instance's copy (OriginSecret above).
    const crCode = lambda.Code.fromAsset(CUSTOM_RESOURCES_DIR);
    const ownFunction = (id: string, handler: string, description: string, timeout: Duration): lambda.Function => new lambda.Function(this, `${id}Function`, {
      runtime: lambda.Runtime.NODEJS_24_X,
      architecture: lambda.Architecture.ARM_64,
      handler,
      code: crCode,
      timeout,
      memorySize: 128,
      logGroup: new logs.LogGroup(this, `${id}Logs`, { retention: logs.RetentionDays.ONE_MONTH, removalPolicy: RemovalPolicy.DESTROY }),
      description: `FactoryOS ${name}: ${description}`,
    });
    const originSecretFn = ownFunction('OriginSecret', 'origin-secret.handler',
      'generates the origin secret into its two encrypted SSM parameters', Duration.minutes(1));
    const ssmParameter = (region: string, parameterName: string) => `arn:${Aws.PARTITION}:ssm:${region}:${this.account}:parameter${parameterName}`;
    originSecretFn.addToRolePolicy(new iam.PolicyStatement({
      actions: ['ssm:PutParameter', 'ssm:DeleteParameter'],
      resources: [ssmParameter(this.region, originSecretParameterName(name)), ssmParameter(EDGE_REGION, edgeOriginSecretParameterName(name))],
    }));
    const originSecret = new CustomResource(this, 'OriginSecret', {
      serviceToken: originSecretFn.functionArn,
      resourceType: 'Custom::OriginSecret',
      serviceTimeout: Duration.minutes(2),
      properties: {
        SecretParameter: originSecretParameterName(name),
        SecretRegion: this.region,
        EdgeParameter: edgeOriginSecretParameterName(name),
        EdgeRegion: EDGE_REGION,
      },
    });

    // ----- The edge function, the distribution ------------------------------------------------------------
    const edge = createEdgeAuth(this, props.edgeStack, name);

    const distribution = new cloudfront.Distribution(this, 'Distribution', {
      comment: `FactoryOS ${name}`,
      priceClass: cloudfront.PriceClass.PRICE_CLASS_100,
      httpVersion: cloudfront.HttpVersion.HTTP2_AND_3,
      enableLogging: true,
      logBucket: accessLogs,
      logFilePrefix: 'cloudfront/',
      // The cookies are the session's tokens.
      logIncludesCookies: false,
      defaultBehavior: {
        origin: origins.VpcOrigin.withEc2Instance(instance, {
          httpPort: 80,
          protocolPolicy: cloudfront.OriginProtocolPolicy.HTTP_ONLY,
          readTimeout: Duration.seconds(60),
        }),
        viewerProtocolPolicy: cloudfront.ViewerProtocolPolicy.HTTPS_ONLY,
        allowedMethods: cloudfront.AllowedMethods.ALLOW_ALL,
        cachePolicy: cloudfront.CachePolicy.CACHING_DISABLED,
        // Everything the viewer sent (WebSocket upgrades for /zenoh and /chat included) except Host.
        originRequestPolicy: cloudfront.OriginRequestPolicy.ALL_VIEWER_EXCEPT_HOST_HEADER,
        responseHeadersPolicy: cloudfront.ResponseHeadersPolicy.SECURITY_HEADERS,
        compress: true,
        edgeLambdas: [{ eventType: cloudfront.LambdaEdgeEventType.VIEWER_REQUEST, functionVersion: edge.version }],
      },
    });
    pin(distribution, PINNED_IDS.distribution);
    // The edge version the distribution associates sends the edge copy of the secret: it exists before.
    distribution.node.addDependency(originSecret);
    // VPC Block Public Access: in an account-region with BPA on (block-ingress), CloudFront cannot put the VPC
    // origin's endpoints into a subnet that BPA covers ("You can't update an endpoint group that has endpoints in a
    // subnet with Block Public Access enabled"). The exclusion is for the instance's own subnet only. That subnet is
    // private: it has no route to the internet gateway, so the exclusion opens no internet path, and the instance
    // stays reachable only from CloudFront's VPC origin (OriginIngress). With BPA off it changes nothing. It exists
    // before the VPC origin.
    const originSubnetBpa = new ec2.CfnVPCBlockPublicAccessExclusion(this, 'OriginSubnetBpaExclusion', {
      internetGatewayExclusionMode: 'allow-bidirectional',
      // The instance's subnet: ec2.Instance takes the first subnet of its selection (into its network interface).
      subnetId: vpc.selectSubnets({ subnetType: ec2.SubnetType.PRIVATE_WITH_EGRESS }).subnets[0].subnetId,
      tags: [{ key: 'Name', value: `${name}-factoryos-vpc-origin` }],
    });
    for (const vpcOrigin of distribution.node.findAll().filter((c) => c instanceof cloudfront.CfnVpcOrigin)) {
      vpcOrigin.node.addDependency(originSubnetBpa);
    }
    const siteDomain = distribution.distributionDomainName;
    const url = `https://${siteDomain}/`;

    // ----- Cognito ----------------------------------------------------------------------------------------
    // Native users only (the installer creates them, never CloudFormation), no self sign-up, TOTP MFA required.
    const pool = new cognito.UserPool(this, 'UserPool', {
      userPoolName: `${name}-factoryos`,
      selfSignUpEnabled: false,
      signInAliases: { email: true },
      signInCaseSensitive: false,
      autoVerify: { email: true },
      standardAttributes: { email: { required: true, mutable: true } },
      mfa: cognito.Mfa.REQUIRED,
      mfaSecondFactor: { otp: true, sms: false },
      passwordPolicy: {
        minLength: 14,
        requireLowercase: true,
        requireUppercase: true,
        requireDigits: true,
        requireSymbols: true,
        tempPasswordValidity: Duration.days(7),
      },
      accountRecovery: cognito.AccountRecovery.EMAIL_ONLY,
      featurePlan: cognito.FeaturePlan.ESSENTIALS,
      userInvitation: {
        emailSubject: 'Your FactoryOS sign-in',
        emailBody: [
          'You have been given access to FactoryOS (Physical AI Production Designer).',
          `<br><br>Open ${url} and sign in with`,
          '<br>user name: {username}',
          '<br>temporary password: {####}',
          '<br><br>At the first sign-in you choose your own password and set up an authenticator app (TOTP) for',
          'multi-factor authentication. The temporary password expires in 7 days.',
        ].join('\n'),
      },
      deletionProtection: false,
      // The uninstall deletes the pool with the stack.
      removalPolicy: RemovalPolicy.DESTROY,
    });
    pin(pool, PINNED_IDS.userPool);

    const domain = pool.addDomain('UserPoolDomain', {
      cognitoDomain: { domainPrefix: domainPrefix(name, this.account, this.region) },
      managedLoginVersion: cognito.ManagedLoginVersion.NEWER_MANAGED_LOGIN,
    });
    pin(domain, PINNED_IDS.userPoolDomain);
    const cognitoHost = `${domain.domainName}.auth.${this.region}.amazoncognito.com`;

    // The client refers to the distribution (callback and logout URLs) and nothing of the distribution's refers to
    // the client or the pool: the edge function gets them from SSM at run time (EdgeConfig below). No cycle.
    const client = pool.addClient('UserPoolWebClient', {
      userPoolClientName: `${name}-web`,
      generateSecret: false,
      authFlows: { userSrp: true },
      oAuth: {
        flows: { authorizationCodeGrant: true },
        scopes: [cognito.OAuthScope.OPENID, cognito.OAuthScope.EMAIL, cognito.OAuthScope.PROFILE],
        callbackUrls: [`https://${siteDomain}/_auth/callback`],
        logoutUrls: [url],
      },
      supportedIdentityProviders: [cognito.UserPoolClientIdentityProvider.COGNITO],
      preventUserExistenceErrors: true,
      enableTokenRevocation: true,
      // Refresh-token rotation (RefreshTokenRotation Feature ENABLED; CDK then leaves out ALLOW_REFRESH_TOKEN_AUTH,
      // which rotation does not allow): every refresh_token grant of the token endpoint returns a new refresh token,
      // valid for what remains of the original's 12 h, and the old one stops working after the grace period.
      // https://docs.aws.amazon.com/cognito/latest/developerguide/amazon-cognito-user-pools-using-the-refresh-token.html#using-the-refresh-token-rotation
      refreshTokenRotationGracePeriod: REFRESH_ROTATION_GRACE,
      accessTokenValidity: Duration.minutes(60),
      idTokenValidity: Duration.minutes(60),
      refreshTokenValidity: SESSION_REFRESH_TTL,
      authSessionValidity: Duration.minutes(15),
    });
    pin(client, PINNED_IDS.userPoolClient);

    const branding = new cognito.CfnManagedLoginBranding(this, 'LoginBranding', {
      userPoolId: pool.userPoolId,
      clientId: client.userPoolClientId,
      useCognitoProvidedValues: true,
    });
    branding.node.addDependency(domain);

    // ----- The edge function's configuration (SSM, us-east-1) ---------------------------------------------
    const crLogs = new logs.LogGroup(this, 'CustomResourceLogs', {
      retention: logs.RetentionDays.ONE_MONTH,
      removalPolicy: RemovalPolicy.DESTROY,
    });
    const edgeConfigName = edgeConfigParameterName(name);
    // Create never overwrites: an existing parameter is another installation's (the same stack name in another
    // Region of this account, whose sign-in this one would rewire to its own user pool), so the deploy fails on it
    // (ParameterAlreadyExists) instead. The value names the Region that wrote it. Only an update (this
    // installation's own parameter, physical id = its name) overwrites.
    const edgeConfigCall = (overwrite: boolean): cr.AwsSdkCall => ({
      service: 'SSM',
      action: 'putParameter',
      region: EDGE_REGION,
      parameters: {
        Name: edgeConfigName,
        Type: 'String',
        Overwrite: overwrite,
        Description: `FactoryOS ${name}: the CloudFront gate's Cognito configuration (written by the stack ${name} in ${this.region})`,
        Value: this.toJsonString({
          format: 1,
          region: this.region,
          userPoolId: pool.userPoolId,
          clientId: client.userPoolClientId,
          cognitoDomain: cognitoHost,
          distributionDomain: siteDomain,
          refreshTtlSeconds: SESSION_REFRESH_TTL.toSeconds(),
        }),
      },
      physicalResourceId: cr.PhysicalResourceId.of(edgeConfigName),
    });
    const edgeConfig = new cr.AwsCustomResource(this, 'EdgeConfig', {
      onCreate: edgeConfigCall(false),
      onUpdate: edgeConfigCall(true),
      // Deletes the parameter its physical id names: its own after a create that succeeded. After a failed create
      // (the collision above) the rollback's delete carries the handler's log stream name instead, which names no
      // parameter this stack may delete: the other installation's parameter is left alone.
      onDelete: {
        service: 'SSM',
        action: 'deleteParameter',
        region: EDGE_REGION,
        parameters: { Name: new cr.PhysicalResourceIdReference() },
        ignoreErrorCodesMatching: EDGE_CONFIG_DELETE_IGNORED,
      },
      policy: cr.AwsCustomResourcePolicy.fromStatements([new iam.PolicyStatement({
        actions: ['ssm:PutParameter', 'ssm:DeleteParameter'],
        resources: [`arn:${Aws.PARTITION}:ssm:${EDGE_REGION}:${this.account}:parameter${edgeConfigName}`],
      })]),
      installLatestAwsSdk: false,
      logGroup: crLogs,
    });

    // ----- Gate check: VerifyAnonymousDenial --------------------------------------------------------------
    const verifyLogs = new logs.LogGroup(this, 'VerifyAnonymousDenialLogs', {
      retention: logs.RetentionDays.ONE_MONTH,
      removalPolicy: RemovalPolicy.DESTROY,
    });
    const verifyFn = new lambda.Function(this, 'VerifyAnonymousDenialFunction', {
      runtime: lambda.Runtime.NODEJS_24_X,
      architecture: lambda.Architecture.ARM_64,
      handler: 'index.handler',
      code: lambda.Code.fromAsset(VERIFY_SOURCE_DIR),
      timeout: Duration.minutes(14),
      memorySize: 256,
      logGroup: verifyLogs,
      description: `FactoryOS ${name}: checks that the CloudFront gate denies anonymous requests`,
    });
    // Everything the gate's behaviour depends on that is known at synth: its code, its settings, the check's code.
    const configHash = crypto.createHash('sha256')
      .update(fs.readFileSync(path.join(EDGE_SOURCE_DIR, 'index.mjs')))
      .update(edgeSettings(name))
      .update(fs.readFileSync(path.join(VERIFY_SOURCE_DIR, 'index.mjs')))
      .update(JSON.stringify({ refresh: SESSION_REFRESH_TTL.toSeconds() }))
      .digest('hex');
    const verify = new CustomResource(this, 'VerifyAnonymousDenial', {
      serviceToken: verifyFn.functionArn,
      resourceType: 'Custom::VerifyAnonymousDenial',
      serviceTimeout: Duration.minutes(16),
      properties: {
        DistributionDomain: siteDomain,
        ClientId: client.userPoolClientId,
        CognitoDomain: cognitoHost,
        Issuer: `https://cognito-idp.${this.region}.amazonaws.com/${pool.userPoolId}`,
        Region: this.region,
        EdgeVersionArn: edge.versionArn,
        ConfigHash: configHash,
      },
    });
    pin(verify, PINNED_IDS.verify);
    verify.node.addDependency(edgeConfig, distribution, branding);

    // ----- Only now: the origin's ingress ------------------------------------------------------------------
    // A function and role of its own (not the AwsCustomResource singleton EdgeConfig runs with): the role may only
    // describe security groups, and the one that writes the gate's configuration may not.
    const originSgFn = ownFunction('VpcOriginSecurityGroupLookup', 'vpc-origin-sg.handler',
      'finds the security group of CloudFront\'s VPC origins in the stack\'s VPC', Duration.minutes(2));
    originSgFn.addToRolePolicy(new iam.PolicyStatement({
      actions: ['ec2:DescribeSecurityGroups'],
      resources: ['*'],
    }));
    const originSg = new CustomResource(this, 'VpcOriginSecurityGroupLookup', {
      serviceToken: originSgFn.functionArn,
      resourceType: 'Custom::VpcOriginSecurityGroup',
      serviceTimeout: Duration.minutes(3),
      properties: { VpcId: vpc.vpcId, GroupName: VPC_ORIGINS_SECURITY_GROUP },
    });
    originSg.node.addDependency(verify, distribution);

    const ingress = new ec2.CfnSecurityGroupIngress(this, 'OriginIngress', {
      groupId: securityGroup.securityGroupId,
      ipProtocol: 'tcp',
      fromPort: 80,
      toPort: 80,
      sourceSecurityGroupId: originSg.getAttString('GroupId'),
      description: 'FactoryOS backend from CloudFront VPC origins (added after VerifyAnonymousDenial)',
    });
    ingress.overrideLogicalId(PINNED_IDS.originIngress);
    ingress.node.addDependency(verify);

    // ----- Outputs (deployment/CONTRACT.md) ----------------------------------------------------------------
    new CfnOutput(this, 'Url', { value: url, description: 'FactoryOS' });
    new CfnOutput(this, 'UserPoolId', { value: pool.userPoolId });
    new CfnOutput(this, 'UserPoolClientId', { value: client.userPoolClientId });
    new CfnOutput(this, 'UserPoolDomain', { value: `https://${cognitoHost}` });
    new CfnOutput(this, 'InstanceId', { value: instance.instanceId });
    new CfnOutput(this, 'SourceBucket', { value: sourceBucket.bucketName });
    new CfnOutput(this, 'ReleaseDocument', { value: releaseDocument.ref });
    new CfnOutput(this, 'LogGroup', { value: logGroup.logGroupName });
    new CfnOutput(this, 'GateVerified', {
      value: verify.getAttString('Verified'),
      description: 'true once VerifyAnonymousDenial passed (only then is the origin opened)',
    });

    logProviderFunctions(this);
    acknowledgeMain(this, { cfg, instance, role, distribution, pool, originSg: originSgFn });
  }

  /** Two AZs without a context lookup (a synth needs no AWS credentials): CloudFormation picks them at deploy. */
  get availabilityZones(): string[] {
    return [Fn.select(0, Fn.getAzs()), Fn.select(1, Fn.getAzs())];
  }
}

/**
 * What the SSM agent and the CloudWatch agent of the instance need, instead of the managed policies:
 *  - Session Manager / Run Command channels (every ssmmessages and ec2messages action, named: they have no
 *    resource-level permissions, OpenDataChannel's session resource aside, which is the service's);
 *  - the agent's own calls of AmazonSSMManagedInstanceCore (registration, associations, inventory, compliance,
 *    patching), on this account's and Region's instances, associations and documents where IAM supports a resource
 *    (the instance's own ARN would be a cycle), and on * where it supports none; no ssm:GetParameter(s) on *:
 *    the role reads only the release parameters (ReleaseParameters) and the instance's origin secret (OriginSecret);
 *  - the stack's log group only (the agent ships /var/log/factoryos/*.log there, user-data.sh);
 *  - PutMetricData in the agent's namespace only. No ec2:DescribeTags: the agent's configuration appends no
 *    dimensions and names no tags.
 */
export function instanceAgentStatements(stack: Stack, logGroup: logs.LogGroup): iam.PolicyStatement[] {
  const p = Aws.PARTITION;
  const { region, account } = stack;
  return [
    new iam.PolicyStatement({
      sid: 'SsmChannels',
      // ssmmessages:* and ec2messages:* spelled out (all of their actions; no wildcard action).
      actions: [
        'ssmmessages:CreateControlChannel', 'ssmmessages:CreateDataChannel', 'ssmmessages:OpenControlChannel', 'ssmmessages:OpenDataChannel',
        'ec2messages:AcknowledgeMessage', 'ec2messages:DeleteMessage', 'ec2messages:FailMessage', 'ec2messages:GetEndpoint',
        'ec2messages:GetMessages', 'ec2messages:SendReply',
      ],
      resources: ['*'],
    }),
    new iam.PolicyStatement({
      sid: 'SsmAgent',
      actions: [
        'ssm:UpdateInstanceInformation', 'ssm:ListInstanceAssociations', 'ssm:DescribeAssociation', 'ssm:GetDocument',
        'ssm:DescribeDocument', 'ssm:UpdateAssociationStatus', 'ssm:UpdateInstanceAssociationStatus', 'ssm:PutComplianceItems',
      ],
      resources: [
        `arn:${p}:ec2:${region}:${account}:instance/*`,
        `arn:${p}:ssm:${region}:${account}:association/*`,
        `arn:${p}:ssm:${region}:${account}:document/*`,
        // AWS-owned documents (AWS-RunShellScript, AWS-RunPatchBaseline, …) have no account in their ARN.
        `arn:${p}:ssm:${region}::document/*`,
      ],
    }),
    new iam.PolicyStatement({
      sid: 'SsmAgentAccount',
      actions: ['ssm:ListAssociations', 'ssm:PutInventory', 'ssm:GetManifest', 'ssm:GetDeployablePatchSnapshotForInstance'],
      resources: ['*'],
    }),
    new iam.PolicyStatement({
      sid: 'AgentLogs',
      actions: ['logs:CreateLogStream', 'logs:PutLogEvents', 'logs:DescribeLogStreams'],
      // The log group (DescribeLogStreams) and its streams (`<group arn>:*`, CreateLogStream / PutLogEvents).
      resources: [`arn:${p}:logs:${region}:${account}:log-group:${logGroup.logGroupName}`, logGroup.logGroupArn],
    }),
    new iam.PolicyStatement({
      sid: 'AgentMetrics',
      actions: ['cloudwatch:PutMetricData'],
      resources: ['*'],
      conditions: { StringEquals: { 'cloudwatch:namespace': AGENT_METRICS_NAMESPACE } },
    }),
  ];
}

/**
 * The agentic planner's Bedrock calls (InvokeModel / InvokeModelWithResponseStream; Converse uses the same actions):
 * the named cross-Region inference profiles in this account and Region, and their foundation models, in whichever
 * Region the profile routes to, only through those profiles (bedrock:InferenceProfileArn).
 */
export function bedrockStatements(stack: Stack, profileIds: readonly string[]): iam.PolicyStatement[] {
  if (!profileIds.length) return [];
  const p = Aws.PARTITION;
  const actions = ['bedrock:InvokeModel', 'bedrock:InvokeModelWithResponseStream'];
  const profiles = profileIds.map((id) => `arn:${p}:bedrock:${stack.region}:${stack.account}:inference-profile/${id}`);
  const models = [...new Set(profileIds.map(foundationModelId))].map((m) => `arn:${p}:bedrock:*::foundation-model/${m}`);
  return [
    new iam.PolicyStatement({ sid: 'BedrockInferenceProfiles', actions, resources: profiles }),
    new iam.PolicyStatement({
      sid: 'BedrockFoundationModels',
      actions,
      resources: models,
      conditions: { StringEquals: { 'bedrock:InferenceProfileArn': profiles } },
    }),
  ];
}

/**
 * The functions of CDK's own custom resource providers (CustomResourceProvider: the S3 auto-delete of the access-log
 * bucket, EdgeFunction's cross-Region ARN reader) log to /aws/lambda/<function>, a group Lambda would create itself,
 * without a retention and outside the stack. Each gets a log group of the stack instead (LoggingConfig.LogGroup), 30
 * days, deleted with the stack; their roles' AWSLambdaBasicExecutionRole already allows writing to it. The log groups
 * are encrypted as the stack's other ones are, by CloudWatch Logs (no customer managed key).
 */
export function logProviderFunctions(stack: Stack): void {
  for (const c of stack.node.findAll()) {
    if (!(c instanceof CfnResource) || c.cfnResourceType !== 'AWS::Lambda::Function' || c instanceof lambda.CfnFunction) continue;
    const provider = c.node.scope as Construct;
    const logGroup = new logs.LogGroup(stack, `${provider.node.id.replace(/[^A-Za-z0-9]/g, '')}Logs`, {
      retention: logs.RetentionDays.ONE_MONTH,
      removalPolicy: RemovalPolicy.DESTROY,
    });
    c.addPropertyOverride('LoggingConfig', { LogGroup: logGroup.logGroupName });
  }
}

function pin(construct: Construct, logicalId: string): void {
  (construct.node.defaultChild as unknown as { overrideLogicalId(id: string): void }).overrideLogicalId(logicalId);
}
