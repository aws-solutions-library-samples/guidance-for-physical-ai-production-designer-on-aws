# Operating the Guidance for Physical AI Production Designer on AWS

This guide complements the [README](../README.md). The README has what you need to deploy, validate, run and remove
the Guidance; this guide has the installer's other tasks, the options, the permissions for a narrower role, the
troubleshooting and the cleanup without the installer. Every command runs from the repository's root, in AWS
CloudShell or in a shell with credentials for the account, like the [Deployment
Steps](../README.md#deployment-steps).

## Contents

1. [Installer commands and options](#installer-commands-and-options)
2. [Manage users](#manage-users)
3. [Install from a release zip](#install-from-a-release-zip)
4. [Update an installation](#update-an-installation)
5. [Rotate the origin secret](#rotate-the-origin-secret)
6. [Provide umarell as a tarball](#provide-umarell-as-a-tarball)
7. [Install without input](#install-without-input)
8. [Bootstrap the AWS CDK yourself](#bootstrap-the-aws-cdk-yourself)
9. [Permissions the installer needs](#permissions-the-installer-needs)
10. [Troubleshooting](#troubleshooting)
11. [Cleanup details](#cleanup-details)

## Installer commands and options

`./deployment/install.sh --help` lists every command and option. The commands:

| Command | What it does |
| --- | --- |
| `install --from-checkout [options]` | installs, or updates to, the checkout `install.sh` is in |
| `install <s3 uri of a release zip> [options]` | installs, or updates to, that release ([Install from a release zip](#install-from-a-release-zip)) |
| `status [--wait]` | the URL, the versions, the release build's progress and the users; `--wait` polls every minute until the release is `ready`, `failed` or `rolled-back` |
| `add-user <email>...`, `remove-user <email>...` | invites or deletes users ([Manage users](#manage-users)) |
| `uninstall [--snapshot] [--edge-only] [--confirm-stack <name>]` | deletes the deployment ([Cleanup](../README.md#cleanup)) |

The options of every command:

- `--region <region>`: the stack's Region. The default is `$AWS_REGION` (CloudShell sets it) or your AWS CLI
  configuration; a Region you did not give with `--region` is printed and asked about.
- `--stack-name <name>`: the stack name, default `factoryos`. One name serves one Region: the installer refuses a
  name that is installed in another Region, because the sign-in function at the edge belongs to the name.
- `--yes`: do not ask before going on. Destructive steps still need their own confirmation.

The options of `install`:

- `--users <email,...>`: the users to invite.
- `--instance-type <type>`: an AWS Graviton instance type, default m7g.2xlarge. The pinned Kiro CLI build is arm64
  only.
- `--volume-gb <n>`: the size of the instance's volume, default 200.
- `--bedrock` or `--no-bedrock`: allow the instance to invoke the agentic planner's models on Amazon Bedrock (the
  cross-Region inference profiles of those models in the stack's Region); off by default.
- `--bedrock-models <id,...>`: the Amazon Bedrock inference profile ids the instance may invoke instead, at most 20,
  no ARNs or wildcards; implies `--bedrock`.
- `--kiro-secret <secret name>`: an AWS Secrets Manager secret in the same Region with a Kiro API key for the
  planning agent (`''` for none). Read the [Security](../README.md#security) section first:
  every user can read that secret.
- `--umarell-tarball <s3 uri>`: umarell from a tarball instead of GitHub ([Provide umarell as a
  tarball](#provide-umarell-as-a-tarball)).
- `--umarell-jam-tarball <s3 uri or file>`: the umarell commit the umarell_jam planner builds on, from a tarball
  instead of GitHub (optional; [Provide umarell as a tarball](#provide-umarell-as-a-tarball)). An update keeps the
  installed one; `''` drops it.
- `--force`: rebuild the installed version.
- `--build-project <name> --build-bucket <bucket>`: compile the release in that AWS CodeBuild project, with the
  export staged in that bucket, instead of on the instance. The installer stack passes its own (`<installer
  stack>-build` and its inputs bucket); without them the instance compiles the release, which takes a few hours.
- `--rotate-origin-secret`: replace the origin secret; the backend restarts twice on the way, so run it when
  nobody is working ([Rotate the origin secret](#rotate-the-origin-secret)). Without it, an update keeps the
  installed one.
- `--allow-downgrade`: install an older version than the installed one.
- `--allow-replace`: allow an infrastructure change that replaces or removes the instance, its volume, the source
  bucket or the user pool. Their data is lost.

An update keeps the installed values of `--instance-type`, `--volume-gb`, `--bedrock`, `--bedrock-models`,
`--kiro-secret` and `--umarell-jam-tarball` unless you give them again.

## Manage users

Invite another user, or delete one. Neither redeploys anything:

```sh
./deployment/install.sh add-user b@example.com
./deployment/install.sh remove-user b@example.com
```

Each invited user gets an e-mail from Amazon Cognito with the URL, their user name and a temporary password, valid
for 7 days, and enrols multi-factor authentication at the first sign-in ([Deployment
Steps](../README.md#deployment-steps), step 4). `add-user` leaves an existing user alone and sends no new e-mail: if a
temporary password expired before its first use, run `remove-user` and then `add-user` for that address. The designs,
cells and plans belong to the deployment, not to a user, so deleting a user deletes none of them.

Cognito's default e-mail sender sends at most about 50 invitations a day. For more users, configure Amazon SES as
the user pool's e-mail sender.

## Install from a release zip

A release can also be installed from its zip, `factoryos-<release>.zip` (one top directory `factoryos-<release>/`),
with its checksum file `factoryos-<release>.zip.sha256`. Upload both to an S3 bucket in your account, then unpack the
zip and run the installer it contains with the zip's S3 URI:

```sh
aws s3 cp s3://<bucket>/factoryos-<release>.zip /tmp/ && cd /tmp && unzip -q factoryos-<release>.zip
./factoryos-<release>/deployment/install.sh install s3://<bucket>/factoryos-<release>.zip --region us-east-1 --users you@example.com
```

The installer checks the zip against the `.sha256` file next to it, and refuses a zip of another version than its
own.

## Update an installation

Run the same install command again with the new version. In the clone, `git pull` and then step 4 of the [Deployment
Steps](../README.md#deployment-steps); from a release zip, the command above with the new zip. The installer finds
the installed stacks and updates them in place:

- It refuses a version older than the installed one (`--allow-downgrade` installs it anyway), and does nothing for
  the installed version (`--force` rebuilds it).
- It shows the infrastructure change and asks before it applies it. It refuses a change that would replace or
  remove the instance, its volume, the source bucket or the user pool (`--allow-replace` allows it, and their data
  is lost).
- A release that changes the instance's user data (its first-boot setup) makes CloudFormation stop and start the
  instance during the update: a few minutes without the application, a running build or planning job is
  interrupted, the volume and its data stay. The installer says so before it asks.
- It keeps the installed options unless you give them again ([Installer commands and
  options](#installer-commands-and-options)).
- The running release keeps serving while the new one builds. A release that fails its health check after the
  switch is switched back, and `status` shows `rolled-back`. The designs, cells and plans on the instance are kept.

To deploy your own changes to `source/`, commit them and run the same install command: the installer packages the
checkout (`git archive` of `HEAD`). A checkout with uncommitted changes is packaged as it is on disk, with a
`+dirty-` version suffix.

## Rotate the origin secret

The edge function adds a secret header to every request it lets through, and the backend refuses a request from
the network without it, so that only this deployment's CloudFront distribution reaches the instance (the README's
[Security](../README.md#security) section). The installer makes the secret when it installs
(48 random bytes) and keeps it in two encrypted AWS Systems Manager parameters: `/factoryos/<stack>/edge-origin-secret`
in US East (N. Virginia), which the edge function reads, and `/factoryos/<stack>/origin-secret` in the Region of the
deployment, which the instance reads. It never appears in a template, in the user data or in a log. An update keeps
it. To replace it, for example after someone left who had a terminal on the instance:

```sh
./deployment/install.sh install --from-checkout --rotate-origin-secret
```

(or the release zip's install command with `--rotate-origin-secret`; it also works with the version that is
installed). The installer rotates in three steps, so that no request is refused for a wrong origin secret: the
instance accepts the new and the old value, the edge function switches to the new value (the installer waits about
7 minutes, until every copy of the edge function has reloaded it), and then the instance accepts only the new one.
The whole rotation takes about 8 minutes, and with the installed version nothing is rebuilt. It is not without
downtime: the backend restarts twice (a few seconds each; the planner router restarts with it), so open terminals
close, viewers reconnect, and a running planning job is stopped. Run it when nobody is working.

If the rotation stops before it finishes (an error, Ctrl-C, a closed terminal during the wait), the old value is
still accepted. The installer says so: run the same install command with `--rotate-origin-secret` again, which
finishes it. Until you do, an update refuses to run without `--rotate-origin-secret`, and a run with the installed
version warns. If you run it with your own role, it also needs `ssm:GetParameter` with decryption and
`ssm:PutParameter` on the two parameters, and `ssm:DescribeParameters` (which IAM cannot narrow to a parameter).

## Provide umarell as a tarball

The release build fetches umarell (the U3 and agentic planners) from GitHub at the commit in
`source/deploy/umarell.pin`. While that repository is not public, pass an
archive of that commit instead. Make it with `git archive --prefix=umarell/` in a clone that has the commit, and
upload it to S3 in your account:

```sh
git -C <umarell clone> archive --prefix=umarell/ -o /tmp/umarell.tar.gz "$(awk '!/^#/ && NF {print $1; exit}' source/deploy/umarell.pin)"
aws s3 cp /tmp/umarell.tar.gz s3://<bucket>/umarell.tar.gz
```

Then add `--umarell-tarball s3://<bucket>/umarell.tar.gz` to the install command. Without it, the release build
fails when it installs the planners.

**The umarell_jam planner** (the planner router's fast weld planner: it reuses a cache of certified weld motions
and plans a prepared cell in seconds) is built on another, older umarell commit, the one in
`source/deploy/umarell-jam.pin`. While the repository is not public, pass that commit as a second, optional archive,
the same way (a local file works too):

```sh
git -C <umarell clone> archive --prefix=umarell/ -o /tmp/umarell-jam.tar.gz "$(awk '!/^#/ && NF {print $1; exit}' source/deploy/umarell-jam.pin)"
```

and add `--umarell-jam-tarball /tmp/umarell-jam.tar.gz` (or an `s3://` copy). Without it the release still installs:
`status` shows `incomplete: umarell_jam`, and umarell_jam's jobs are planned by its fallback, umarell_u3 (minutes
instead of seconds). The first umarell_jam job on a cell also prepares that cell's cache (minutes, at most about
half an hour; what it could not prepare in that time umarell_u3 plans, and the next job continues the preparation);
later jobs on the same cell take seconds.

## Install without input

`deployment/deploy.sh` runs the same installation without any input, from the clone it is in: a new stack name per
run, the static parameters at its top, and `--yes`. It checks the sign-in gate and waits for the release.
`deployment/cleanup.sh` removes that installation again. Until the umarell repository is public, set
`UMARELL_TARBALL` at the top of `deploy.sh`, or export `ONECLICK_UMARELL_TARBALL`; it stops at once otherwise.
`UMARELL_JAM_TARBALL` (`ONECLICK_UMARELL_JAM_TARBALL`) is umarell_jam's, optional: without it, it only warns.
The CloudFormation installer runs the same scripts with `ONECLICK_*` overrides (deployment/README.md lists them).
The two scripts are for automated validation. For your own deployment, use the [Deployment Steps](../README.md#deployment-steps).
[deployment/README.md](../deployment/README.md) describes the installer, the AWS CDK app and their tests.

## Bootstrap the AWS CDK yourself

`install.sh install` checks the AWS CDK bootstrap in the Region of the deployment and in US East (N. Virginia)
(`us-east-1`), and bootstraps a Region only where it is missing. To bootstrap yourself, with the CDK CLI version the
Guidance pins, run this from the repository's root:

```sh
(cd deployment && npm ci) && deployment/node_modules/.bin/cdk bootstrap aws://<account-id>/us-east-1
```

Run the `cdk bootstrap` command once more with your Region in place of `us-east-1` if you deploy to another Region.
Do not run it in `deployment/`: there the CLI first synthesizes the Guidance's app, which needs the installer's
context, and stops with an error.

## Permissions the installer needs

An administrator role in the account covers everything the installer does. For a narrower role, the installer
needs:

- AWS CloudFormation and the AWS CDK bootstrap roles (`cdk-hnb659fds-*`): to bootstrap where needed, deploy, review
  change sets, list the resources of and delete the two stacks. The bootstrap itself creates IAM roles, an S3
  bucket, an Amazon ECR repository and an SSM parameter. `ssm:GetParameter` on that parameter,
  `/cdk-bootstrap/hnb659fds/version`, in the Region of the deployment and in us-east-1, is how the installer finds
  out whether a Region is bootstrapped (without it, it bootstraps again on every install).
- `sts:GetCallerIdentity`; `ec2:DescribeInstances` and `ec2:DescribeVolumes` (the installed instance and its
  volume); `ssm:GetParameter` on the public Ubuntu AMI parameter
  `/aws/service/canonical/ubuntu/server/24.04/stable/current/arm64/hvm/ebs-gp3/ami-id`.
- `ssm:GetParameter`, `ssm:PutParameter` and `ssm:DeleteParameter` (`uninstall`) on `/factoryos/<stack>/*` in the
  Region of the deployment (including the origin secret `/factoryos/<stack>/origin-secret`, read with decryption
  through the AWS managed key `aws/ssm`); `ssm:GetParameter` on `/factoryos/<stack>/edge-config` in us-east-1, and
  `ssm:GetParameter`, `ssm:PutParameter` and `ssm:DeleteParameter` (`uninstall`) on
  `/factoryos/<stack>/edge-origin-secret` there ([Rotate the origin secret](#rotate-the-origin-secret));
  `ssm:DescribeParameters` (on `*`: IAM cannot narrow it; the installer reads the origin secret parameter's
  description with it, to find a rotation that did not finish); `ssm:DescribeInstanceInformation`; and
  `ssm:SendCommand` and `ssm:GetCommandInvocation` with the stack's `<stack>-Release` document on its instance.
- `s3:PutObject` in the stack's source bucket (`releases/*`, `vendor/*`), `s3:GetObject` there on `vendor/*` (an
  update checks that the installed umarell_jam tarball is still there), and `s3:GetObject` on the release zip and
  the umarell tarballs you install from S3. For `uninstall`: `s3:ListBucketVersions` and `s3:DeleteObjectVersion`
  there.
- `cognito-idp:AdminCreateUser`, `cognito-idp:AdminDeleteUser` and `cognito-idp:ListUsers` on the stack's user pool.
- `logs:GetLogEvents` on the stack's log group (`status`).
- For `uninstall`: `logs:DeleteLogGroup` on the log groups Lambda creates outside the stacks,
  `/aws/lambda/<stack>-*` in the Region of the deployment and `/aws/lambda/us-east-1.<stack>-edge-*` in every
  Region, and on the stack's own groups (`<stack>-*` in the Region of the deployment);
  `cloudformation:ListStackResources` on both stacks; and `ec2:DescribeRegions`. For `uninstall --snapshot`: `ec2:CreateSnapshot`, `ec2:CreateTags` (on
  snapshots) and `ec2:DescribeSnapshots`.

## Troubleshooting

- **The release ends `failed`.** `./deployment/install.sh status` shows the error and the last lines of the build
  log. The full log is in the log group of the stack's `LogGroup` output, stream `<instance id>/release.log`. The
  previous release, if any, keeps serving. Fix the cause and run the install command again, with `--force` for the
  same version.
- **The installer stack fails in its release build.** With **Release build** `codebuild` the release is compiled in
  the installer's `<installer stack>-build` project; its `ReleaseBuildLogs` output links to the builds and their logs.
  Nothing was sent to the instance. Fix the cause and update the installer stack again, or set **Release build** to
  `instance` to compile on the instance.
- **The release build fails when it installs the planners.** umarell could not be fetched: install with
  `--umarell-tarball` ([Provide umarell as a tarball](#provide-umarell-as-a-tarball)).
- **`status` shows `incomplete: umarell_jam`.** The release serves, but the umarell_jam planner is not set up, and its
  jobs fall back to umarell_u3. The release log has the reason (`WARNING: umarell_jam is not set up: …`). Usually its
  umarell commit could not be fetched: install again with `--umarell-jam-tarball` ([Provide umarell as a
  tarball](#provide-umarell-as-a-tarball)), with `--force` for the same version.
- **`status` shows `rolled-back`.** The new release failed its health check after the switch, and the previous
  release serves again. `status` and the build log show why.
- **The URL signs you in and then shows an error page.** The first release is not `ready` yet: the installer stack
  completes when it is. Follow it with `./deployment/install.sh status --wait`.
- **The CloudShell session ended during the build.** The build goes on without it. Run `status --wait` again.
- **The installer asks to confirm the Region, or refuses the stack name.** Give the Region with `--region`. A stack
  name installed in another Region is refused: install the second deployment with another `--stack-name`.
- **An invitation did not arrive, or its temporary password expired.** Check the spam folder, then see [Manage
  users](#manage-users). Cognito's default sender sends about 50 invitations a day.
- **The installer bootstraps the AWS CDK on every install.** Your role lacks `ssm:GetParameter` on
  `/cdk-bootstrap/hnb659fds/version` ([Permissions the installer needs](#permissions-the-installer-needs)).
- **The edge stack does not delete.** CloudFront removes the Lambda@Edge function's replicas only some time after the
  distribution is gone. Run `./deployment/install.sh uninstall --edge-only` again after about 1 hour.
- **The installer refuses `--instance-type`.** It has to be an AWS Graviton (arm64) type, such as m7g.2xlarge: the
  pinned Kiro CLI build is arm64 only.

## Cleanup details

### What `uninstall` deletes

`./deployment/install.sh uninstall` lists what it deletes and asks you to type the stack name (`--confirm-stack
<name>` confirms it without a terminal). It empties the stack's versioned source bucket, then deletes both stacks
with everything in them: the instance and its volume (every design, cell and plan on it), the user pool and its
users, the CloudFront distribution, the VPC with its NAT gateway, the source and access log buckets, the log groups,
and the release parameters `/factoryos/<stack>/*`. It also deletes the log groups Lambda created outside the stacks:
those of the stack's custom resource functions (`/aws/lambda/<stack>-Custom*`) and, once the edge stack is deleted,
those of the sign-in function's Lambda@Edge replicas (`/aws/lambda/us-east-1.<stack>-edge-*`) in every Region.
Last, it deletes the log groups the stack owned (`<stack>-CustomResourceLogs*`, `<stack>-VerifyAnonymousDenialLogs*`,
`<stack>-VpcFlowLogs*`) once more, about 30 seconds after the stack is gone: a custom resource logs while the stack
is deleted, and that late log line re-creates its group, empty and without retention. `--snapshot` keeps an EBS
snapshot of the volume first.

### What stays

Delete these yourself, if they apply:

- The EBS snapshot of `--snapshot`, once you no longer need it:
  `aws ec2 delete-snapshot --snapshot-id <snapshot id>` (the installer printed the id).
- The log groups Lambda created outside the stacks, if the uninstall warned that it could not delete them, or if you
  removed the stacks without the installer: `/aws/lambda/<stack>-Custom*` (the stack's custom resource functions:
  the S3 auto-delete function and, in a Region other than us-east-1, the cross-Region parameter reader) in the
  Region of the deployment, and `/aws/lambda/us-east-1.<stack>-edge-*` (the Lambda@Edge replicas) in each Region
  where CloudFront ran the sign-in function. Delete them in the CloudWatch console of each Region.
- The release zip and the umarell tarball you uploaded to your own bucket.

The AWS CDK bootstrap stacks (`CDKToolkit`) stay: they belong to the account and are shared by every AWS CDK app in
it.

### Without the installer

The installation's stacks are an AWS CDK app, so the equivalent of `uninstall` is `cdk destroy --all` from
`deployment/`. Empty the source bucket first: it is versioned and kept from deletion while it holds objects (empty it
with **Empty** in the Amazon S3 console). The app needs its context to synthesize:

```sh
cd deployment && npm ci
npx cdk destroy --all -c stackName=factoryos -c amiId="$(aws ssm get-parameter \
  --name /aws/service/canonical/ubuntu/server/24.04/stable/current/arm64/hvm/ebs-gp3/ami-id \
  --query Parameter.Value --output text)"
```

Then delete the parameters the installer wrote, `/factoryos/factoryos/release/wanted`,
`/factoryos/factoryos/release/state` and `/factoryos/factoryos/install-options`
(`aws ssm delete-parameter --name <name>`). If the edge stack does not delete yet, retry the `cdk destroy` after
about 1 hour, and then delete what [stays](#what-stays).
