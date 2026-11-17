# deployment/: the installer and the AWS CDK app

This directory deploys the Guidance into an AWS account: `install.sh`, the installer the Guidance README's
[Deployment Steps](../README.md#deployment-steps) use, and the AWS CDK app (TypeScript) it deploys. To install the
Guidance, follow the README; this page is for developing and testing the deployment itself. The names its parts share
(stacks, outputs, parameters, paths on the instance, the archive and its `VERSION.json`) are in
[CONTRACT.md](CONTRACT.md).

## What it deploys

Two AWS CloudFormation stacks ([the README's architecture](../README.md#overview)):

- `<stack>` (default `factoryos`) in the Region of the deployment:
  - a VPC in two Availability Zones, with private subnets for the instance and one NAT gateway for its outbound
    traffic, and VPC flow logs;
  - one AWS Graviton Amazon EC2 instance (m7g.2xlarge, Ubuntu 24.04 arm64, 200 GB encrypted gp3, IMDSv2 with a hop
    limit of 1), without a public IP address, SSH masked, reached by administrators only through AWS Systems Manager;
  - the source bucket (`releases/`, `vendor/`) the instance builds from, and an access log bucket;
  - an Amazon Cognito user pool (native users only, no self sign-up, TOTP MFA required, managed login) with a public
    app client (PKCE, no secret);
  - an Amazon CloudFront distribution with a VPC origin to the instance on port 80;
  - the `<stack>-Release` SSM document that starts a release on the instance, and the CloudWatch log group of the
    bootstrap and release logs;
  - the `VerifyAnonymousDenial` custom resource, which checks that the edge gate denies anonymous requests before the
    instance's security group lets CloudFront in (the `GateVerified` output).
- `<stack>-edge` in us-east-1: the Lambda@Edge viewer-request function that checks the Cognito session on every
  request. It reads its configuration from the SSM parameter `/factoryos/<stack>/edge-config` in us-east-1.

The release is compiled from `source/` (no prebuilt binaries are published): by the CloudFormation installer's
release-build CodeBuild project (Arm, Ubuntu 24.04, `bootstrap/build-bundle.sh`), or, without one (`install.sh`
alone, or `ReleaseBuild` `instance`), by the instance itself. The instance installs it with the host setup of
`source/deploy/`.

## Layout

| Path | What |
|---|---|
| `install.sh` | the installer's entry point (`install.sh --help`); the logic is in `installer/` |
| `installer/{lib,preflight,infra,release,users,status,uninstall}.sh` | arguments and helpers; the account, Region and stack checks; the archive and the CDK app (bootstrap, deploy, change review); the release step; the users; `status`; `uninstall` |
| `deploy.sh`, `cleanup.sh` | the one-click scripts: a whole installation and its removal without any input, for automated validation; the CloudFormation installer runs them too |
| `pai-pd-installer.yaml` | the CloudFormation installer: a CodeBuild project that gets the Guidance (GitHub, S3, HTTPS or an upload) and runs `deploy.sh` / `cleanup.sh` |
| `make-artifacts.sh` | writes an artifact set for the CloudFormation installer: the export, umarell and (optionally) umarell_jam's umarell with their `.sha256` and an `artifacts.json` |
| `bin/app.ts`, `lib/` | the CDK app: `app.ts` (both stacks, cdk-nag), `config.ts` (the context, validated), `factoryos-stack.ts`, `edge.ts`, `nag.ts` (the accepted cdk-nag findings, each with its reason) |
| `lambda/edge-auth/` | the Lambda@Edge gate (Cognito session cookies, PKCE sign-in, refresh, logout, header hygiene); its configuration is loaded at INIT, never on a request (a viewer request has 5 s) |
| `lambda/verify-denial/` | the `VerifyAnonymousDenial` custom resource |
| `bootstrap/` | the instance side: `user-data.sh` (first boot), `install-build-deps.sh` (the build dependencies), `build-bundle.sh` (the release build in the installer's CodeBuild project, on its Ubuntu 24.04 arm64), `release-handoff.sh` (run by the `<stack>-Release` document), `factoryos-release.sh` and `factoryos-release.service` (the release build, deploy, health check and rollback) |
| `test/` | the CDK app's tests (vitest): the stacks, the edge gate's denial matrix, the custom resource; and `smoke.py`, the post-install smoke test |
| `tests/` | the one-click scripts' tests (`test-oneclick.sh`); the CloudFormation installer's build script, function and `make-artifacts.sh` (`test-installer-template.sh`) |
| `installer/tests/`, `bootstrap/tests/` | the installer's and the release's shell tests |
| `CONTRACT.md` | the interfaces between the installer, the CDK app and the instance |

The CDK context cache (`cdk.context.json`), `node_modules/` and `cdk.out/` are never committed (`.gitignore`).

## Develop and test

Node.js 20 or newer. Everything below runs offline, without AWS credentials:

```sh
cd deployment
npm ci
npm test            # tsc --noEmit, then vitest: the stacks (cdk-nag included), the edge gate, the custom resource
npm run synth       # both stacks into cdk.out/, with a dummy account and AMI; cdk-nag reports into cdk.out/
bash installer/tests/test-installer.sh    # the installer, with fake aws, npx, unzip and git on PATH
bash bootstrap/tests/test-release.sh      # the release on the instance, in a temporary root
bash tests/test-oneclick.sh </dev/null    # deploy.sh and cleanup.sh, with a fake clone and installer
```

A synth with other context, as the installer passes it (CONTRACT.md, "Stacks"), still offline:

```sh
env CDK_DEFAULT_ACCOUNT=111122223333 CDK_DEFAULT_REGION=us-east-1 \
  npx cdk synth --all -c amiId=ami-1234567890abcdef0 -c instanceType=m7g.2xlarge -c bedrock=true
```

cdk-nag's AWS Solutions rules and CloudFormation Validate run on every synth as Validations plugins. A finding that
`lib/nag.ts` does not acknowledge, with its reason, is printed and listed in `cdk.out/validation-report.json`; an
Error-level one fails the synth, a warning does not. CI (`package:extras`, the public export's verify-build and its
security scan) fails on any unacknowledged finding in that report, warnings included, so check it before you commit:

```sh
node -e 'const r=require("./cdk.out/validation-report.json");for(const p of r.pluginReports)console.log(p.pluginName,(p.violations||[]).length,"unacknowledged")'
```

The tests also pin the logical IDs of the instance, its volume, the source bucket and the user pool, and
check that the user data does not depend on the release version, so that an update never replaces them.

**After an installation** (from a workstation, against an installed stack): `test/smoke.py` checks the denial matrix,
signs in with a temporary user (TOTP enrolled, deleted afterwards), and checks the viewer, the WebSockets, the tools,
the umarell_jam planner when the release ships it (on the instance, as `factoryos`: its plain umarell 76ea13a and its
venv, `python -m umarell_jam ... setup` within 180 s into a temporary root, and `describe_planner umarell_jam` without
the gate's setup warnings; `--jam-cell <cell>` also checks it is available for that cell; a release that ships it
without it set up fails) and that the instance listens only on loopback and port 80 (again after Start Simulation,
unless `--skip-sim`). Its
pinned requirements are in `test/requirements-smoke.txt`, and it needs Chromium for Playwright:

```sh
python3 -m venv /tmp/smoke-venv && /tmp/smoke-venv/bin/pip install -r test/requirements-smoke.txt
/tmp/smoke-venv/bin/python -m playwright install chromium
/tmp/smoke-venv/bin/python -I test/smoke.py --self-test          # offline: a stub gate on 127.0.0.1, no AWS
/tmp/smoke-venv/bin/python -I test/smoke.py --stack-name factoryos --region us-east-1 [--skip-sim]
```

The real run needs credentials for the stack's account (`cloudformation:DescribeStacks`, `cognito-idp:Admin*User` and
`ListUsers` on the pool, `ssm:SendCommand` and `GetCommandInvocation` on the instance); `--help` lists the options.

## The installer

`install.sh` runs in AWS CloudShell or on any machine with the AWS CLI v2, Node.js 20 or newer, python3, tar and unzip
(and git for `--from-checkout`). Every command first prints the account, the Region and the stack name, and asks before
it goes on (`--yes` does not ask). A Region that does not come from `--region` is printed and asked about.

| Command | What it does |
|---|---|
| `install --from-checkout [options]` | packages the checkout `install.sh` is in (`git archive` of `HEAD` when it is clean, the files on disk otherwise) and installs it, or updates the installation to it |
| `install s3://<bucket>/<key>.zip [options]` | the same, from a release archive in S3 (`.zip` or `.tar.gz`, one top directory; its `<uri>.sha256` is checked when it exists) |
| `status [--wait]` | the URL, the stacks, whether the gate is verified, the wanted and the running version, the release phase with the last lines of its log (and, for a `ready` release, what it provisioned without: `incomplete: umarell_jam`), the users; `--wait` follows the release until `ready`, `failed` or `rolled-back` |
| `add-user <email>...`, `remove-user <email>...` | invites or deletes users of the user pool (only once `GateVerified` is `true`) |
| `uninstall [--snapshot] [--edge-only] [--confirm-stack <name>]` | empties the source bucket and deletes both stacks, the parameters and the log groups Lambda created outside the stacks, after listing what goes and a typed confirmation of the stack name (or `--confirm-stack`, for a run without a terminal); `--edge-only` retries only the edge stack |

**What an invited user can do.** Every user of the pool is the workspace owner and may open a terminal in the viewer:
a shell on the instance as `factoryos`, with the instance role through the instance metadata service. That includes
the stack's source bucket, writing `/factoryos/<stack>/release/state`, the allowed Bedrock models and, with
`--kiro-secret`, reading the Kiro API key secret (the terminals keep `KIRO_API_KEY` out of the shell's environment,
but `aws secretsmanager get-secret-value` returns it). Invite only people you would give that access to.

An install of a new stack: the CDK bootstrap where it is missing (the Region and us-east-1), `npm ci` of this app in
a work directory (`/tmp/factoryos-install/<version>`, as CloudShell's home holds only 1 GB), `cdk deploy --all` with
the installation's context, the release step (the archive into the source bucket, `/factoryos/<stack>/release/wanted`,
the `<stack>-Release` document), and the users once `GateVerified` is `true`. An update reviews the change sets first
and refuses a downgrade, a no-op without `--force`, and a change that would replace the instance, its volume, the
source bucket or the user pool without `--allow-replace` (a change of the instance's user data alone is not one: the
change set marks the instance `Conditional`, and CloudFormation stops and starts it; the installer warns). Before either, it refuses a stack name whose edge
configuration belongs to another Region.

The release builds on the instance in `factoryos-release.service`, which outlives the installer: `status` picks it up
again at any time.

**umarell_jam** (the cache-based umarell weld planner of the planner router) needs the plain umarell commit of
`source/deploy/umarell-jam.pin` (76ea13a) besides umarell's own. While sample-umarell is not public, pass it as
`--umarell-jam-tarball <s3://… | file>` (`git archive --format=tar.gz --prefix=umarell/
76ea13aafdeede10be11936cf0830613e8717907 > umarell-jam.tar.gz` in a clone that has the commit); the installer checks
it before anything changes, uploads it to `vendor/umarell-jam-<sha12>.tar.gz` of the source bucket and records it in
the installation's options, so an update keeps it (`--umarell-jam-tarball ''` drops it). It is optional: without it,
or when anything of umarell_jam's setup fails on the instance, the release still ends `ready`, `status` shows
`incomplete: umarell_jam`, the release log has a `WARNING: umarell_jam is not set up: …` line, and umarell_jam's jobs
fall back to umarell_u3 (minutes instead of seconds). The next update provisions it again.

## The one-click scripts

`deploy.sh` and `cleanup.sh` are thin wrappers around `install.sh` for the automated validation of the Guidance
(AWS CodeBuild, Amazon Linux 2023): no input, `AWS_REGION=us-east-1`, and their static parameters at the top
(`STACK_NAME`, a new one per run; `ADMIN_EMAIL`; `REPO_URL`; `WAIT_FOR_RELEASE`; `RELEASE_TIMEOUT_SECONDS`;
`UMARELL_TARBALL`; `UMARELL_JAM_TARBALL`).

- `deploy.sh`: checks the tools (Node.js 20 or newer, npm, the AWS CLI v2, git, python3, curl; no timeout(1), so it
  runs on stock macOS too), gets the account, and takes the installer of the tree it is `deployment/deploy.sh` of (a
  clone, the staging repo, a CodeBuild source: the revision under test), or, as a standalone copy (ASCV's
  BuilderSpace copy), clones `REPO_URL` (`git clone --depth 1`). With `UMARELL_TARBALL=''` it checks that the pin's
  umarell URL answers `git ls-remote` anonymously and stops at once otherwise (the release would fail hours later).
  With `UMARELL_JAM_TARBALL=''` and a `deploy/umarell-jam.pin` whose URL is not public it only warns (umarell_jam is
  optional: the release installs without it); a set `UMARELL_JAM_TARBALL` goes to `--umarell-jam-tarball`.
  It checks that a `CDKToolkit` stack is absent or healthy but never bootstraps itself (`cdk bootstrap` in
  `deployment/` synthesizes the app first, which needs the installer's context; `install.sh` bootstraps where
  `/cdk-bootstrap/hnb659fds/version` is missing). It runs `./deployment/install.sh install --from-checkout --yes
  --region "$AWS_REGION" --stack-name "$STACK_NAME" --users "$ADMIN_EMAIL"`, checks `GateVerified` and that
  anonymous requests get `401` and `302`, waits for the release (`install.sh status --wait`, at most
  `RELEASE_TIMEOUT_SECONDS`) when `WAIT_FOR_RELEASE=1`, and records the stack name in `factoryos-oneclick.state` in
  the directory it runs in.
- `cleanup.sh`: reads that file, takes the installer the same way (its own tree, else `deploy.sh`'s clone, else a new
  clone), runs `./deployment/install.sh uninstall --yes --confirm-stack "$STACK_NAME"` (which also deletes the
  log groups Lambda created outside the stacks), retries `uninstall --edge-only` every 15 minutes for up to 2 hours
  while CloudFront still holds the Lambda@Edge replicas, and checks with `describe-stacks` that both stacks are gone.

Run both from the same directory: `bash deployment/deploy.sh`, then `bash deployment/cleanup.sh`.

The CloudFormation installer overrides the static parameters with `ONECLICK_*` variables. The prefix is their own:
CodeBuild sets `AWS_REGION` itself, and that must not move an ASCV run out of us-east-1.

| Variable | Overrides | Script |
|---|---|---|
| `ONECLICK_REGION` | the Region (`us-east-1`) | both |
| `ONECLICK_STACK_NAME` | `STACK_NAME`; for `cleanup.sh`, instead of the state file | both |
| `ONECLICK_STATE_FILE` | `STATE_FILE` | both |
| `ONECLICK_ADMIN_EMAIL`, `ONECLICK_UMARELL_TARBALL`, `ONECLICK_UMARELL_JAM_TARBALL`, `ONECLICK_WAIT_FOR_RELEASE`, `ONECLICK_RELEASE_TIMEOUT_SECONDS` | the static values of the same name | `deploy.sh` |
| `ONECLICK_INSTANCE_TYPE`, `ONECLICK_VOLUME_GB`, `ONECLICK_BEDROCK` (`true`/`false`) | `--instance-type`, `--volume-gb`, `--bedrock`/`--no-bedrock` (empty: the installer's defaults) | `deploy.sh` |
| `ONECLICK_BUILD_PROJECT`, `ONECLICK_BUILD_BUCKET` | `--build-project`, `--build-bucket`: the release built in that CodeBuild project (empty: on the instance) | `deploy.sh` |

## The CloudFormation installer

`pai-pd-installer.yaml` is self-contained: a customer uploads it in the CloudFormation console, and nothing else is
needed.

**What it creates:**
- `InstallerProject`: a CodeBuild project (Amazon Linux 2023 `standard:5.0`, Node.js 20, 8 h, one build at a time).
  Its inline buildspec writes and runs `/tmp/pai-pd-build.sh`.
- `InstallerRole`: the project's role. It has `AdministratorAccess` and trusts only CodeBuild, for this project.
- `ReleaseBuildProject` (`<stack>-build`): the release build (Arm `BUILD_GENERAL1_XLARGE`, 32 vCPUs and 64 GiB,
  `public.ecr.aws/ubuntu/ubuntu:24.04`, 4 h, one build at a time). `install.sh` starts it with the export staged under
  `builds/` of the inputs bucket; its artifacts go back there (deployment/CONTRACT.md "Release build in AWS
  CodeBuild"). `ReleaseBuildRole` reads and writes `builds/*` of the inputs bucket and its log stream only.
- `InputsBucket`: a private bucket. It is the upload target, HTTPS downloads are staged in it for `install.sh`, and
  after a successful install it keeps that export as `installed/<AppStackName>.<ext>`. An uninstall uses that copy, so
  it works after a presigned URL expired or `latest` moved on. The bucket is emptied and deleted with the stack.
- `HelperFunction`: one Python function behind three custom resources:
  - `StartBuild` starts the create's build with `WAIT_HANDLE_URL`, starts a new build on an update that changes a
    parameter, and stops running builds on delete.
  - `ReadOutputs` copies `Url` and `GateVerified` from the application stack.
  - `StopReleaseBuilds` stops a running release build on delete.
  - `EmptyInputsBucket` empties the bucket on delete.
- The `InstallDone` wait condition, which waits up to 8 h 10 min.
- Log groups for the build (`/pai-pd/installer/<stack>`) and the function, so that nothing is left behind.

**The build script:**
1. Resolves the export. An explicit `SourceArchiveUri` comes first. Then the artifact set: `ArtifactsUri`, or the
   inputs bucket for `SourceMode=upload`, using `artifacts.json` or the names `pai-pd*` / `factoryos*`
   `.zip`/`.tar.gz`, `umarell*.tar.gz` and `umarell-jam*.tar.gz` (umarell_jam's, optional; `UmarellJamTarballUri` wins
   over it). Last, GitHub: the tag that `<repo>/releases/latest` redirects to, or
   `Release`, as `<repo>/archive/refs/tags/<tag>.tar.gz`.
2. Checks every `.sha256` next to an `s3://` file.
3. Unpacks the export (exactly one top directory, with `deployment/`).
4. Stages an `https://` umarell (or umarell_jam) archive to `s3://<inputs>/staging/`, because the one-click path
   passes `s3://` only.
5. Runs that export's `deployment/deploy.sh` (`ACTION=install`) or `deployment/cleanup.sh` (`ACTION=uninstall`) with
   the `ONECLICK_*` variables.
6. On exit, when `WAIT_HANDLE_URL` is set, signals `SUCCESS` with the URL as `Data`, or `FAILURE` with the last
   `ERROR` line and the build ID as `Reason`.

`SourceMode=upload` has no wait condition and no `StartBuild`: the user starts the build after the upload.

Test the template with `bash deployment/tests/test-installer-template.sh`, which runs the build script and the
function against fakes and runs cfn-lint where it is installed. For an end-to-end run, make a set with
`make-artifacts.sh` from an export zip (`scripts/export-public.py`), the umarell tarball and, for umarell_jam, its
umarell tarball (`--umarell-jam`). Then create the stack in
a test account with `SourceMode=upload` (upload the set) or `SourceMode=s3` (`ArtifactsUri`).
