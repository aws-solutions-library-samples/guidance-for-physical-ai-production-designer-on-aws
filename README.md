# Guidance for Physical AI Production Designer on AWS

## Table of Contents

1. [Overview](#overview)
    - [What the installer deploys](#what-the-installer-deploys)
    - [Cost](#cost)
2. [Prerequisites](#prerequisites)
3. [Deployment Steps](#deployment-steps)
4. [Deployment Validation](#deployment-validation)
5. [Running the Guidance](#running-the-guidance)
6. [Next Steps](#next-steps)
7. [Cleanup](#cleanup)
8. [FAQ, known issues, additional considerations, and limitations](#faq-known-issues-additional-considerations-and-limitations)
9. [Revisions](#revisions)
10. [Notices](#notices)
11. [Authors](#authors)

## Overview

Programming a new robot cell still passes through a separate tool and a hand-off for every step: the product data,
the robots and their tools, the cell layout, the motion plan, the interlocks, and the PLC and robot programs. Each
new product variant starts much of that work again. This Guidance shows how to bring these steps into one
browser-based application on AWS, Physical AI Production Designer (FactoryOS in the code), which an AI agent can
drive through MCP tools and whose output is deterministic, checked control code:

1. **Robo Dresser** mounts a tool on a robot (URDF and meshes) and computes the dressed robot's collision hulls.
2. **Robo Fitter** lays the dressed robots, fixtures and a floor plan out into a cell, and checks which robot reaches
   which weld.
3. **Product & Task** holds the product to weld and the welding task on it.
4. **Planning** assigns the welds to the robots and plans every motion: learned policies (hierarchical reinforcement
   learning and PPO, the checkpoints included) and a scripted generator for the reference cell they were trained on;
   umarell's planning tools (OMPL) driven by a deterministic parallel orchestrator (U3) for any cell; optionally an
   agentic planner that takes constraints in plain words (with models on Amazon Bedrock). In the viewer, a Kiro CLI
   agent runs the planners from its chat through the planner router's MCP tools. Every plan becomes a behaviour tree
   and is checked before it is offered: joint and speed limits, robot-robot and robot-environment collisions, and the
   timing of every motion.
5. **Behaviour tree executor** runs the plan on the simulated cell (Pinocchio kinematics, Coal collision checks) in
   the 3D view.
6. **Control code**: plan2cell generates the head PLC's program (IEC 61131-3 Structured Text, for CODESYS) and the
   robots' KUKA KRL programs from the plan.

The sample data is a reference welding cell, `welding_cell_2_small`: two KUKA KR 240 R2900 robots with stick welding
tools and 15 spot welds.

### What the installer deploys

The CloudFormation installer (`deployment/pai-pd-installer.yaml`) deploys one shared workspace for the deployment's
users on one Amazon EC2 instance. It runs the AWS CDK app in `deployment/` in AWS CodeBuild, which creates two AWS
CloudFormation stacks: `factoryos` in the Region you choose, and `factoryos-edge` in US East (N. Virginia), where
CloudFront's Lambda@Edge function lives.

| Part | In this Guidance's deployment |
| --- | --- |
| Amazon CloudFront and Lambda@Edge | CloudFront serves the viewer. The Lambda@Edge function checks the Amazon Cognito tokens on every request, including WebSocket upgrades, before CloudFront forwards it. |
| Amazon Cognito | The `factoryos` stack's own user pool, in the Region of the deployment: managed login, multi-factor authentication required, users created by the deployer, no self sign-up and no federation. |
| Amazon EC2 | One AWS Graviton instance (m7g.2xlarge, Ubuntu 24.04) in a private subnet of the stack's VPC, which CloudFront reaches through a VPC origin on port 80. It has no public IP address and reaches the internet only outbound, through a NAT gateway, for the build dependencies and the Kiro service. |
| The workspace | Shared by every user, on the instance: the backend with the web viewer, the Robo Dresser, Robo Fitter and Product & Task, the cell runtime, the Kiro agent, the planner router and planners, and plan2cell. Its Amazon EBS volume holds the designs, cells and plans. |
| Amazon Bedrock | Optional: the agentic planner invokes Anthropic models on Amazon Bedrock when the installer's **Amazon Bedrock** parameter is `true`. |
| Amazon S3 | One bucket holds the release sources the instance builds from, a second one the access logs of that bucket and of CloudFront. |
| AWS Systems Manager | The only administrative access (Session Manager, no SSH). Run Command starts release builds, and Parameter Store keeps the release state. |

Amazon CloudWatch collects the instance's bootstrap and release (build) logs, the VPC flow logs and the Lambda logs.
Virtual robot controllers and PLCs, GPU simulation hosts, policy training and an on-premises release pipeline are not
part of this deployment: plans run on the simulated cell in the viewer, and the learned policies' checkpoints are
included.

### Cost

_You are responsible for the cost of the AWS services used while running this Guidance. As of November 2026, the cost
for running this Guidance with the default settings in the US East (N. Virginia) Region is approximately $301.32 per
month for running one deployment around the clock for a team of up to 10 users._

_We recommend creating a [Budget](https://docs.aws.amazon.com/cost-management/latest/userguide/budgets-managing-costs.html)
through [AWS Cost Explorer](https://aws.amazon.com/aws-cost-management/aws-cost-explorer/) to help manage costs. Prices
are subject to change. For full details, refer to the pricing webpage for each AWS service used in this Guidance._

The Amazon EC2 instance is most of the cost, and it is billed for every hour it runs, whether or not anyone uses it.
Stopping it outside working hours (for example, running it 200 hours a month) lowers its share from $238.27 to about
$65.28; the EBS volume and the NAT gateway are billed while it is stopped. Do not stop it while a release builds
(the installer stack's `BuildLogs` output shows the build). The installer's uninstall removes every billed resource the
Guidance created ([Cleanup](#cleanup)); the AWS CDK bootstrap stacks (`CDKToolkit`, with an S3 staging bucket) that the
installer creates where they are missing, in your Region and in us-east-1, stay and cost a few cents a month.

The table prices every line at the On-Demand list price without the AWS Free Tier, as the AWS Pricing Calculator
does, except Amazon Cognito's 10,000 free monthly active users, which the calculator subtracts too. On a real bill,
the always-free tiers of Amazon CloudFront (1 TB of data transfer out to the internet and 10,000,000 requests per
month: all of its line except the $0.20 to the origin) and the 100 GB of data transfer out to the internet per month
usually apply as well, so you pay about $295 a month instead of $301.32. Lambda@Edge has no free tier. Outside
us-east-1 its log groups have no retention period: their storage grows by about $0.015 a month until you uninstall.

### Sample Cost Table

The following table provides a sample cost breakdown for deploying this Guidance with the default parameters in the
US East (N. Virginia) Region for one month. The prices are the public On-Demand list prices of that Region.

| AWS service  | Dimensions | Cost [USD] |
| ----------- | ------------ | ------------ |
| Amazon EC2 | 1 m7g.2xlarge On-Demand Linux instance (8 vCPU, 32 GiB), 730 hours a month | $ 238.27 |
| Amazon EBS | 200 GB gp3 volume (3,000 IOPS and 125 MB/s included) | $ 16.00 |
| Amazon VPC: NAT gateway | 1 NAT gateway, 730 hours, 50 GB of data processed (build dependencies) | $ 35.10 |
| Amazon VPC: public IPv4 address | 1 address of the NAT gateway, 730 hours | $ 3.65 |
| Amazon CloudFront | 50 GB data transfer out to the internet, 1,000,000 HTTPS requests, and 10 GB of data transfer out to the origin (uploads and client-to-server WebSocket frames) per month | $ 5.45 |
| AWS Lambda@Edge | 1,000,000 viewer-request invocations per month, 128 MB, 5 ms each | $ 0.63 |
| Amazon Cognito | 10 monthly active users (Essentials feature plan), TOTP MFA; within the 10,000 free monthly active users | $ 0.00 |
| Amazon S3 | 2 GB Standard storage of release sources and access logs, about 22,000 requests | $ 0.06 |
| Amazon CloudWatch | 2.5 GB of logs ingested per month (the instance's bootstrap and release logs, VPC flow logs as vended logs, and the Lambda@Edge function's logs), kept for a month (stored compressed, about 0.4 GB) | $ 1.26 |
| Amazon EC2: data transfer out | 10 GB to the internet through the NAT gateway | $ 0.90 |
| AWS Systems Manager | Session Manager, Run Command and standard Parameter Store parameters | $ 0.00 |
| AWS Lambda | the stacks' custom resources and the gate check, a few invocations per deployment | $ 0.00 |
| AWS KMS | the AWS managed key `aws/ebs` that encrypts the volume | $ 0.00 |
| AWS CloudFormation | the two stacks, which use AWS resource types only | $ 0.00 |
| **Total** | | **$ 301.32** |
| Amazon Bedrock (optional, not in the total) | Agentic planner: for example 20 planning sessions with 200,000 input and 20,000 output tokens each on Anthropic Claude Sonnet 5 at $2.20 and $11.00 per million tokens, the US geographic cross-Region inference price the deployment's `us.` inference profiles pay (on Claude Opus 5, at $5.50 and $27.50, the same sessions cost $33.00) | $ 13.20 |

## Prerequisites

### Operating System

You need only a web browser: you run the installer from **AWS CloudShell** (it has the AWS CLI, git and the
credentials of your console session) or from the AWS CloudFormation console. The installer creates the instance it
builds and runs the Guidance on: Ubuntu 24.04 LTS (arm64) on an AWS Graviton m7g.2xlarge with a 200 GB gp3 volume.
You do not launch or prepare a host yourself.

### Third-party tools

On the instance, the release build fetches the Rust crates, npm packages, Python packages and Rerun 0.38.1 from
crates.io, the npm registry, PyPI and GitHub, each at the version its lock file or pin names. Their licences are listed
in [THIRD-PARTY-NOTICES.md](THIRD-PARTY-NOTICES.md). Two more come from their own publishers:

- **umarell** (the U3, umarell_jam and agentic planners): fetched from GitHub at the commits pinned in
  `source/deploy/umarell.pin` and `source/deploy/umarell-jam.pin`. That repository is not public yet; until it is, an
  installation needs the installer's **umarell archive** parameters ([Known issues](#known-issues)).
- **Kiro CLI** 2.26.1 (the planning agent): installed on the instance and checked against its sha256. It needs a Kiro
  subscription and is subject to Kiro's terms ([kiro.dev](https://kiro.dev)). Review them before you sign the agent
  in. The viewer plans through this agent's chat for every planner, so planning in the viewer needs it signed in;
  only the agentic planner also needs Amazon Bedrock.

### AWS account requirements

- An AWS account, and permission to create an AWS CloudFormation stack with IAM resources. The installer's AWS
  CodeBuild project does the rest with its own role ([Deployment Steps](#deployment-steps)).
- The m7g.2xlarge instance needs 8 vCPUs of the Amazon EC2 quota described in [Service limits](#service-limits).
- Optional, for the agentic planner: access to Anthropic models on Amazon Bedrock in the Region of the deployment
  (first-time use of Anthropic models may require submitting use case details in the Amazon Bedrock console). The
  instance may call Bedrock only when the installer's **Amazon Bedrock** parameter is `true`, and then only the
  cross-Region inference profiles of the models the agentic planner uses and the models they route to.

### aws cdk bootstrap

The Guidance uses the AWS CDK, which has to be bootstrapped once per account in the Region of the deployment and in
US East (N. Virginia) (`us-east-1`), where the stack of the Lambda@Edge function goes. The installer checks both and
runs `cdk bootstrap` only where the bootstrap is missing, so you need not do it yourself.

### Service limits

The m7g.2xlarge instance needs 8 vCPUs of the **Running On-Demand Standard (A, C, D, H, I, M, R, T, Z) instances**
quota of Amazon EC2 in the Region. The deployment also creates one VPC, one NAT gateway with one Elastic IP address,
one CloudFront distribution and one Cognito user pool. Check the quotas, and request an increase if needed, on the
[Service Quotas console](https://console.aws.amazon.com/servicequotas/).

Cognito's default e-mail sender sends at most about 50 invitations a day; for more users, configure Amazon SES for the
user pool ([Manage users](docs/operations.md#manage-users)).

### Supported Regions

The default and tested Region is US East (N. Virginia) (`us-east-1`). The Guidance works in any commercial Region that
offers AWS Graviton m7g instances, CloudFront VPC origins and Cognito managed login; the edge stack is always in
us-east-1. The agentic planner additionally needs Anthropic models on Amazon Bedrock in the Region of the deployment.
One stack name serves one Region: to install a second deployment in another Region, give it another name (the
installer's **Application stack name** parameter).

## Deployment Steps

The template `deployment/pai-pd-installer.yaml` creates an AWS CodeBuild project and starts it. The project gets
this Guidance, runs its installer without prompts, and reports the result to the stack. The stack completes once
Physical AI Production Designer is ready, which takes about 2.5 to 3 hours. The CodeBuild project's role has
`AdministratorAccess`, because the installer bootstraps the AWS CDK and creates IAM roles, Amazon CloudFront and
Lambda@Edge. Only CodeBuild can assume the role, and only for this stack's project.

1. Open AWS CloudShell in the Region of the deployment, and clone the repository:
   ```sh
   git clone https://github.com/aws-solutions-library-samples/guidance-for-physical-ai-production-designer-on-aws
   ```
2. Create the installer stack. Replace `you@example.com` with the e-mail address of the first user: Amazon Cognito
   sends the sign-in URL and a temporary password to it. `--on-failure DO_NOTHING` keeps a failed stack, with its
   build log and its `Uninstall` command, instead of rolling it back.
   ```sh
   aws cloudformation create-stack --stack-name pai-pd-installer \
     --template-body file://guidance-for-physical-ai-production-designer-on-aws/deployment/pai-pd-installer.yaml \
     --capabilities CAPABILITY_IAM --on-failure DO_NOTHING \
     --parameters ParameterKey=AdminEmail,ParameterValue=you@example.com
   ```
   To install a specific release instead of the latest one, add `ParameterKey=Release,ParameterValue=<tag>`. Until
   the umarell repository is public, also add `ParameterKey=UmarellTarballUri,ParameterValue=<s3:// or https://>`
   and, for the umarell_jam planner, `ParameterKey=UmarellJamTarballUri,ParameterValue=<s3:// or https://>`
   ([Known issues](#known-issues)).

   **In the console instead:**
   1. Download `deployment/pai-pd-installer.yaml`.
   2. In the [AWS CloudFormation console](https://console.aws.amazon.com/cloudformation/home), choose
      **Create stack**, then **With new resources (standard)**, then **Upload a template file**.
   3. Enter **Administrator e-mail** (and the umarell archives, if you need them).
   4. Under **Stack failure options**, choose **Preserve successfully provisioned resources**.
   5. Acknowledge that the stack creates IAM resources, then choose **Submit**.
3. Follow the installation. The stack's `BuildLogs` output links to the build:
   ```sh
   aws cloudformation describe-stacks --stack-name pai-pd-installer --query 'Stacks[0].[StackStatus,Outputs]'
   ```
   The invitation e-mail arrives once the infrastructure is deployed, after about 40 minutes. Until the release build
   on the instance is finished, the URL signs you in and then shows an error page. When the stack is
   `CREATE_COMPLETE`, its `Url` output is the application.
4. Sign in. The e-mail from Cognito has the URL, your user name and a temporary password (valid for 7 days).
   1. Open the URL, sign in with the e-mail address and the temporary password, and choose a new password.
   2. Enrol multi-factor authentication: scan the QR code with an authenticator app (TOTP) and enter its code. Every
      later sign-in asks for a code.
   3. Sign the Kiro agent in (the viewer plans through its chat, for every planner): in the viewer's settings, choose
      how (a personal AWS Builder ID, or your organisation's IAM Identity Center start URL and Region), choose **Sign
      in to the agent**, and approve the request on the page that opens. The agent's sign-in is shared by every user
      of the deployment.

To install another release later, update the installer stack with a new **Release**. The update starts a new build,
which updates the installation in place; follow it in `BuildLogs`.

**Installing from your own files.** The template can also install from files you provide instead of GitHub:

- **Source `upload`**: the stack creates a private bucket and completes in about a minute. Upload the files to it
  (the `UploadTo` output), then start the installation (the `StartInstall` output).
- **Source `s3`**: set **Artifact set** to an `s3://bucket/prefix/` that holds the files.
- **Any source**: set **Export archive** or the **umarell archive** parameters to an `s3://` or `https://` URI, for
  example a presigned URL. Presigned URLs only need to be valid when the build starts, which downloads them first. A
  presigned URL in a parameter can be read by anyone allowed to describe the stack until it expires, so keep its
  expiry short.

`deployment/make-artifacts.sh <export.zip> --umarell <umarell.tar.gz> --out <dir or s3://...>` writes such a set:
the export, the umarell archive, a `.sha256` file for each, and an `artifacts.json` that names them. The build
checks every `.sha256` file before it installs anything.

The [operations guide](docs/operations.md) covers running the deployment: [adding and removing
users](docs/operations.md#manage-users), [updating an installation](docs/operations.md#update-an-installation), and
[troubleshooting](docs/operations.md#troubleshooting).

## Deployment Validation

1. Check the stack's outputs. `GateVerified` is `true` (the sign-in was checked to deny anonymous requests before the
   instance was opened to CloudFront), and `Url` is the Guidance's address:
   ```sh
   aws cloudformation describe-stacks --stack-name factoryos --query "Stacks[0].Outputs"
   ```
   The outputs also name the instance (`InstanceId`), the user pool (`UserPoolId`), the source bucket (`SourceBucket`)
   and the log group (`LogGroup`).
2. Check the installer stack: it is `CREATE_COMPLETE` (or `UPDATE_COMPLETE` after a new **Release**), and its build
   log (`BuildLogs`) ends with the release `ready` and its version:
   ```sh
   aws cloudformation describe-stacks --stack-name pai-pd-installer --query 'Stacks[0].StackStatus'
   ```
3. Check that an anonymous request is denied. With the `Url` output in `URL`, the first command prints `401` and the
   second `302` (a redirect to the Cognito sign-in):
   ```sh
   URL=$(aws cloudformation describe-stacks --stack-name factoryos \
     --query "Stacks[0].Outputs[?OutputKey=='Url'].OutputValue | [0]" --output text)
   curl -sS -o /dev/null -w '%{http_code}\n' "${URL%/}/api/v1/health"
   curl -sS -o /dev/null -w '%{http_code}\n' -H 'Accept: text/html' "${URL%/}/"
   ```
4. Open the URL in your browser and sign in ([Deployment Steps](#deployment-steps), step 4): the Physical AI Production
   Designer viewer loads.

If the release ends `failed`, the build log (`BuildLogs`) shows the error and the last lines of the release build; the
previous release (if any) keeps serving. Fix the cause, then start the installer's build again: open the CodeBuild
project from the `BuildLogs` output and choose **Start build**.
Where the full log is, and other problems: [Troubleshooting](docs/operations.md#troubleshooting).

## Running the Guidance

**Inputs.** The sample data in `source/assets/`, which the release installs on the instance: the reference cell
`welding_cell_2_small` (two KUKA KR 240 R2900 robots with stick welding tools, 15 spot welds), the catalogue it is
built from (the robot `kr240r2900`, the tool `sticktool`, the dressed robot `kr240r2900__sticktool`, the fixtures
`weld_fixture_box`, `weld_cube` and the sample cell's `car_door_fixture`, the products `weld_box` and `weld_cube_urdf`, scenes and a floor plan), and the
planners' checkpoints for that cell.

**Before you start.** Sign in at the stack's URL. Planning runs through the planner agent's chat, so the Kiro agent
must be signed in first ([Deployment Steps](#deployment-steps), step 4); the learned
policies and U3 do not call Amazon Bedrock. The viewer shows the workflow as cards on the left, top to bottom, the 3D
view in the centre, and the open tool's panels on the right. The top bar's **Simple | Advanced** switch shows more settings in Advanced; Simple uses their defaults.
If a step fails, see [Troubleshooting](docs/operations.md#troubleshooting) in the operations guide.

### 1. Product & Task: the part and its welds

Choose **Open Product & Task**, and in the panel's **Catalogue** section choose `weld_box` (it is copied into your
products and opened). To make your own, choose **New**, set its geometry (a box or a mesh file), add a **Spot
welding** task, place its points (**Add operation**, or **Generate a grid of points on a face**) and choose **Save**. The planners never add weld points: they plan the ones a task defines.

![Product & Task: a box product in the 3D view with labelled spot welds on its top face, and the panel with its geometry and the task's table of weld positions and normals](assets/images/walkthrough/product-and-task.jpg)\
*The Product & Task tool, shown with another, larger box product than `weld_box`: the panel and the task table look
the same.*

**Result:** the box in the 3D view with its six spot welds `W01` to `W06` on the top face, and the task `spot_weld_1`
with each weld's position and approach direction.

### 2. Robo Dresser: mount the tool on the robot

Choose **Open Dresser**, pick the robot `kr240r2900` and the tool `sticktool` in the **Catalogue** panel, and choose
**Dress**. **Collision hulls** shows the hulls the collision checks use.

**Result:** "new URDF written", with the path `urdf-dressed-robots/kr240r2900__sticktool/kr240r2900__sticktool.urdf`,
and **Diagnostics** reports no issues: the tool sits on the flange, and the tool centre point (TCP) at its tip.

### 3. Robo Fitter: lay out the cell

In the **Robo Fitter** card, pick the cell `welding_cell_2_small` and choose **Open**. To lay out your own cell
instead: **Open Fitter**, **New scene**, place a product and two `kr240r2900__sticktool` robots from the **Palette**,
set each robot's position and home pose, turn on **Show task**, choose **Check reachability**, then **Save** the scene
and choose **Use as cell**.

**Result:** the two robots, the car-door fixture and the cell's weld points in the 3D view; the card says the cell is open (read-only).
The weld points sit 0.25 m above the fixture's panel, as in the umarell data the cell comes from: the fixture is an
obstacle the robots clear, not a part they touch, and the electrodes stop above the panel at every weld. In
your own scene, a weld turns green when at least one robot reaches it.

### 4. Agentic Planning: plan the motions

In the **Agentic Planning** card, pick the task program (in your own cell, `<product> · <task> (from product)`),
leave the planner on `auto`, and choose **Plan Cell**. The planner chat opens, and the agent calls the planner
router's tools: `recommend_planner` picks a planner (U3, for the sample cell too: the learned policies were trained on
a version of it without the fixture), `task_program_from_products` writes a task program where the cell has none, and `plan`
and `plan_status` plan and report progress. Advanced mode lets you pick the planner, for example `umarell_jam`, umarell's weld planner with a component cache
(its first job on a cell fills the cache and takes a few minutes, later ones take seconds). In the chat, you can ask for
another planner, or add a constraint in plain words, which the agentic planner handles (it calls Amazon Bedrock).

![Agentic Planning: the planner chat beside a cell, with the agent's planner recommendation, its tool calls and the plan's progress](assets/images/walkthrough/agentic-planning.jpg)\
*The planner agent choosing a planner and planning a cell, shown on another cell than `welding_cell_2_small`, for
which the agent picks U3.*

**Result:** the agent's summary names the planner, the plan file, the cycle time and the checks the plan passed
(joint and speed limits, collisions, timing). U3 takes a few minutes.

### 5. Behaviour Tree: replay the plan

In the **Behaviour Tree** card, pick the scene and the behaviour tree, named `bt_<id>_<cell> · <planner> · <n> ops`,
and choose **Load & Play**.

![Behaviour Tree: two robots welding a box in the 3D view, with the playback progress in the card and each robot's operations on the timeline below](assets/images/walkthrough/behaviour-tree.jpg)\
*A plan replayed, shown on another cell than `welding_cell_2_small`: both robots welding, and the timeline of their
operations.*

**Result:** the robots weld in the simulated cell's 3D view. The card shows the progress with **Pause**, **Stop**
and **Loop**, and the timeline under the view shows each robot's operations, to step through and scrub. (**Watch in
Isaac Sim** stays off: this deployment has no Isaac Sim host.)

### 6. Control Code Export: the PLC and robot programs

In the **Generate PLC + robot code** card, pick the behaviour tree and choose **Export PLC + robot code**. Advanced
mode shows the options: **Profile** `Simulation`, **PLC dialect** `CODESYS V3.5`, **Robot language** `KUKA KRL`,
**OPC UA symbols** and **ST type check**. **Show the code** opens the build; **Download archive (.zip)** saves it.

![Control Code Export: the Generate PLC + robot code card with its profile, PLC dialect and robot language and the Export button, next to the build's warnings and its file tree of KRL and PLC files](assets/images/walkthrough/control-code-export.jpg)\
*The export options, and the generated build's warnings and file tree.*

**Result:** the card reports the number of files and warnings; the build opens in place of the 3D view, with its
file tree, each file's code and the **Warnings** (**Back to the 3D view** closes it). The cards after this one (deploy
to a virtual cell, review in the PLC IDE, start production) need virtual controllers, which this deployment does not
include; they stay waiting.

### Expected output

A control code build holds:

- `BUILD.md`: the build's summary, with its `PLAN_ID`, the robots, the interlock zones with their ticket order, and
  the planned makespan;
- `plc/`: the head PLC's CODESYS project in IEC 61131-3 Structured Text (`plc/codesys/head/`, with the function blocks
  `POU/FB_Cell.st` and the zone arbiter `POU/FB_ZoneArbiter.st`), its robot simulators, `plc/project.json` and the
  interlock zones (`plc/ZONES.md`);
- `krl/robot1/`, `krl/robot2/`: each robot's KUKA KRL program, with its `INSTALL.md`;
- `plan2cell-build.json`: the same build as JSON.

`source/plan2cell/examples/welding_cell_2_small/` is the committed build of the sample plan
`source/plan2cell/test_data/bt_1790934549_welding_cell_2_small.json`: 2 robots, 2 interlock zones (`IL01`, `IL02`) and
a planned makespan of 64.297 s. The plan2cell tests check on every build that it is reproduced byte for byte.

### What the output means

The PLC program sequences the two robots through the plan's operations and grants the interlock zones in the planned
ticket order, so the robots never share a zone; the KRL programs are each robot's motions, with their handshakes to
the PLC. They are for the simulated reference cell ([Additional considerations](#additional-considerations)).

## Next Steps

- Add your own robots, tools, fixtures and products to the catalogue (URDF with meshes), dress them in the Robo
  Dresser, and lay out your own cells in the Robo Fitter.
- Plan your own cells with U3 or the agentic planner, and run the plans in the Behaviour Tree Executor.
- Explore the code generator's options (`plan2cell options`), and co-simulate a build's PLC program with the robots
  (`plan2cell simulate`, see `source/plan2cell/README.md`).
- Turn on the agentic planner's Amazon Bedrock access (update the installer stack with **Amazon Bedrock** `true`),
  and sign the agent in.
- Change the code in `source/`, and deploy your own version through the installer's **Source** parameter
  ([Deployment Steps](#deployment-steps), Installing from your own files).
- Operate the deployment: users, updates, options, permissions and troubleshooting are in the [operations
  guide](docs/operations.md).
- Develop and test the deployment itself: [deployment/README.md](deployment/README.md).

## Cleanup

1. Run the installer stack's `Uninstall` command in AWS CloudShell. Alternatively, open the CodeBuild project from
   the `BuildLogs` output, choose **Start build with overrides**, and set the environment variable `ACTION` to
   `uninstall`. The build empties the stack's versioned source bucket, then deletes both stacks with everything in
   them, among them the instance and its volume (every design, cell and plan on it), the user pool and its users, and
   the CloudFront distribution, and the log groups Lambda created outside the stacks ([the full
   list](docs/operations.md#what-uninstall-deletes)). It also retries the edge stack for up to 2 hours while
   CloudFront removes the Lambda@Edge replicas.
2. When the build has succeeded, delete the installer stack. That removes the CodeBuild project, its role and log
   group, the helper function, and the inputs bucket with everything in it. Deleting the installer stack alone does
   **not** remove the application.
   ```sh
   aws cloudformation delete-stack --stack-name pai-pd-installer
   aws cloudformation wait stack-delete-complete --stack-name pai-pd-installer
   ```
3. The AWS CDK bootstrap stacks (`CDKToolkit`) stay: they belong to the account and are shared by every AWS CDK app
   in it.

## FAQ, known issues, additional considerations, and limitations

For problems with the installation, the release build, users or the cleanup, see
[Troubleshooting](docs/operations.md#troubleshooting) in the operations guide.

### Known issues

- **umarell is not public yet.** The U3 and agentic planners need umarell at the commit pinned in
  `source/deploy/umarell.pin` (`08d5a39`), and umarell_jam needs it at `source/deploy/umarell-jam.pin` (`76ea13a`);
  the release build fetches both from GitHub. Until that repository is public, the fetch fails and so does the
  release: give the installer the **umarell archive** (`UmarellTarballUri`) and, for umarell_jam, the **umarell_jam
  archive** (`UmarellJamTarballUri`) ([Deployment Steps](#deployment-steps); how to make them: [Provide umarell as a
  tarball](docs/operations.md#provide-umarell-as-a-tarball)). Without the umarell_jam archive the release still
  installs; umarell_jam is then marked incomplete and its jobs fall back to U3.
- **The first release takes hours.** The instance builds everything from source: the Rust workspaces with link-time
  optimisation, Coal and Pinocchio, the web viewer and the planners' Python environment. Until the first release is
  `ready`, the URL signs you in and shows an error page. An update builds next to the running release, which keeps
  serving.
- **No AWS WAF.** The distribution has no web ACL: every request is authenticated at the edge (Cognito session, Lambda@Edge)
  before it reaches the instance, and the login pages are Cognito's managed login. The AWS CDK app acknowledges the
  cdk-nag rules `AwsSolutions-CFR2` (no WAF on the distribution) and `AwsSolutions-COG8` (the Cognito Essentials
  plan, without the Plus plan's threat protection) with these reasons. To add AWS WAF, associate a web ACL with the
  distribution and consider the Cognito Plus plan; both add cost.
- The pinned Kiro CLI build is arm64 only: the instance type has to be a Graviton type.
- Some third-party crates have published advisories that no release of the crate that depends on them fixes yet. They
  are tracked and will be removed when those crates update:
  - `lz4_flex` 0.10.0, RUSTSEC-2026-0041 (High), through zenoh 1.10.1. It is used only for zenoh's transport
    compression, which is off by default, and the Guidance does not turn it on.
  - `quick-xml` 0.36.2, 0.39.4 and 0.30.0, RUSTSEC-2026-0194 and RUSTSEC-2026-0195 (High), through Rerun's COLLADA
    mesh loader, `urdf-rs` and `behaviortree-rs`. The worst case is slow parsing of a crafted XML file that you open
    yourself; none of these crates uses the namespace reader that RUSTSEC-2026-0195 is about.
  - `rustls` 0.20.9, RUSTSEC-2024-0336 (High), in `source/plan2cell/`, through `trust-runtime` and `tiny_http`. Only
    `tiny_http`'s HTTPS server uses it, and plan2cell runs `trust-runtime` in process without starting a server.
  - `lru` 0.14.0 and 0.16.4, RUSTSEC-2026-0253 (no severity given), through zenoh 1.10.1. It needs cache keys or
    values whose own code panics; zenoh's keys are plain ids.

### Security: who can reach what

- **Anyone** can reach the CloudFront URL, and gets the Cognito sign-in page and nothing else: the Lambda@Edge
  function sends every request without a valid session to the sign-in (`302`), or answers it with `401`, before
  CloudFront forwards it. The installer creates users only after the stack verified that (`GateVerified`).
- **Every user of the user pool** shares the whole deployment: its designs, cells and plans, and its terminals. A
  terminal in the viewer is a shell on the instance as the service user `factoryos`, the user the backend runs as,
  with the permissions of the instance's IAM role through the instance metadata service: its own source bucket, its
  release parameters (including writing the release state that `status` shows), its log group, the Amazon Bedrock
  models you allowed. Users also share the Kiro session and the Bedrock usage, which your account pays for. Invite only
  people you would give that access to.
- **Administrators** reach the instance only through AWS Systems Manager Session Manager (no SSH, no public IP
  address), with IAM permissions for it in the account.
- The instance's security group accepts traffic only from CloudFront's VPC origins, on port 80, and only after the
  sign-in was verified. That rule admits every CloudFront distribution with a VPC origin for the instance, so the
  edge function also adds a secret header (the origin secret) to every request it lets through, and the backend
  refuses (`403`) any request from the network without it: a distribution without this deployment's edge function
  reaches nothing. The backend has no other authentication of its own: never open the instance to other traffic.
  The origin secret is not a boundary against the deployment's own users, whose terminals can read it;
  [Rotate the origin secret](docs/operations.md#rotate-the-origin-secret) replaces it.
- **The terminal also has the Systems Manager and CloudWatch agents' permissions, and they reach past this
  deployment.** The instance role carries what the two agents on the instance need, written out like the
  [`AmazonSSMManagedInstanceCore`](https://docs.aws.amazon.com/aws-managed-policy/latest/reference/AmazonSSMManagedInstanceCore.html)
  managed policy but without its Parameter Store reads. IAM cannot narrow these to the one instance, so they cover
  the whole account in the deployment's Region, and a terminal user has them too: they can read every Systems
  Manager document and association in that Region (your own documents included, with the scripts and parameters
  in them), report association status, compliance and inventory for any managed instance there (so another
  instance could be made to look patched or compliant in Systems Manager), and write metrics into the `CWAgent`
  namespace, also in the name of other instances. They cannot run commands on other instances, read Parameter
  Store, or read other instances' data. If the account or Region runs other workloads, install the Guidance into
  an account of its own.
- The session is Cognito's own tokens, in `__Host-` cookies (HttpOnly, Secure, SameSite=Lax): the ID and access
  tokens are valid for 60 minutes, and the edge function renews them with the refresh token (valid for 12 hours).
  Cognito replaces the refresh token at every renewal (refresh-token rotation), and the old one stops working a
  minute later.
- **Signing out does not end a copied session at once.** The edge function checks the tokens itself (signature,
  issuer, client, expiry) and does not ask Cognito on every request whether they were revoked. Signing out revokes
  the refresh token at Cognito and clears the cookies, but an ID and access token copied out of the browser before
  that stay accepted until they expire, at most 60 minutes later. Amazon Cognito: "revoked tokens will still be
  valid if they are verified using any JWT library that verifies the signature and expiration of the token"
  ([Ending user sessions with token revocation](https://docs.aws.amazon.com/cognito/latest/developerguide/token-revocation.html)).
  The same holds for a user you remove with `remove-user`: their browser keeps access for up to 60 minutes.
- **Encryption in transit has three weaker legs.** Browsers reach CloudFront over HTTPS only, but on the default
  `*.cloudfront.net` domain CloudFront always uses the `TLSv1` security policy, which also accepts TLS 1.0 and 1.1
  ([ViewerCertificate](https://docs.aws.amazon.com/cloudfront/latest/APIReference/API_ViewerCertificate.html),
  [Supported protocols and ciphers](https://docs.aws.amazon.com/AmazonCloudFront/latest/DeveloperGuide/secure-connections-supported-viewer-protocols-ciphers.html));
  current browsers no longer offer those versions, so only old clients are affected. Raising the minimum takes your
  own domain name with an AWS Certificate Manager certificate on the distribution, which this Guidance does not set
  up. From CloudFront to the instance, the traffic is plain HTTP on port 80: it runs through CloudFront's VPC origin
  into the private subnet and never crosses the internet, but it is not encrypted. And the instance installs Ubuntu
  packages over HTTP from the mirrors the Ubuntu image configures: apt checks the archive's signatures, so a package
  cannot be altered unnoticed, but which packages the instance downloads is visible on the way.
- **No Amazon Bedrock guardrail on the agents.** The chat agent (the Kiro CLI) and the agentic planner (Anthropic
  models on Amazon Bedrock) read the shared workspace's cells, products, task programs and plans, and text in those
  files can carry instructions that steer them (prompt injection). The deployment configures no
  [Amazon Bedrock guardrail](https://docs.aws.amazon.com/bedrock/latest/userguide/guardrails.html) and filters
  neither the models' input nor their output. What limits the effect: the agent calls only the tools named in
  `source/deploy/kiro/factoryos-planner.json`; without asking, it can read files, play back the simulated cell, save
  a moved layout as a new scene and run a co-simulation, and every other tool that writes asks you in the chat
  first, so read what it says a call will change before you allow it. The models do not compute the motions
  themselves: planning tools do, and the plans are checked before they are offered. The learned policies' and U3's
  plans pass the planner router's export checks (joint and speed limits, collision clearance) and are marked
  `unverified` when one fails; the LLM planner's plans are checked by umarell's own plan assembly (collisions,
  continuity between the motions), which refuses a plan that fails, and are not checked again by the router. What
  is planned moves no equipment: plans replay in the simulated cell in the viewer, and the control code export is
  files that you validate yourself ([Additional considerations](#additional-considerations)). An injected
  instruction can do at most what the signed-in user can already do with a terminal (above). Treat a cell or
  product file from someone else like code.

### Additional considerations

- `source/cell-planner/runs/` holds pre-trained PyTorch checkpoints (`.pt`) of the learned policies for the reference
  cell, and `source/cell-planner/trajgen/samples/` a precomputed sample plan with its timeline (`.npz`). They are
  binary model and sample files, loaded by the planners; they are not executables.
- The backend reaches the NICE DCV Connection Gateway (the Windows IDE in the browser) on loopback only. It pins the
  self-signed certificate of the DCV Connection Gateway when `FACTORY_OS_DCV_GATEWAY_CERT` names it, as the DCV setup
  does; without it, the backend accepts any certificate on that loopback connection.
- The generated PLC and robot programs are for the simulated reference cell. Review, test and validate any generated
  control code on virtual controllers and under your own safety procedures before it runs on physical equipment.

For any feedback, questions, or suggestions, please use the issues tab under this repo.

## Revisions

All notable changes to this Guidance are documented here, following [Keep a Changelog](https://keepachangelog.com/)
and [Semantic Versioning](https://semver.org/).

- **Unreleased**: the first public version: the design tools, the planners' runtime, plan2cell and the sample cell in
  `source/`, and the installer and AWS CDK app in `deployment/`, with the CloudFormation installer
  `deployment/pai-pd-installer.yaml`.

## Notices

*Customers are responsible for making their own independent assessment of the information in this Guidance. This Guidance: (a) is for informational purposes only, (b) represents AWS current product offerings and practices, which are subject to change without notice, and (c) does not create any commitments or assurances from AWS and its affiliates, suppliers or licensors. AWS products or services are provided “as is” without warranties, representations, or conditions of any kind, whether express or implied. AWS responsibilities and liabilities to its customers are controlled by AWS agreements, and this Guidance is not part of, nor does it modify, any agreement between AWS and its customers.*

References to third-party services or organizations in this Guidance do not imply an endorsement, sponsorship, or
affiliation between Amazon or AWS and the third party. Guidance from AWS is a technical starting point, and you can
customize your integration with third-party services when you deploy the architecture.

This Guidance is licensed under the MIT-0 License. See the [LICENSE](LICENSE) file. The third-party software it builds
on, and their licences: [THIRD-PARTY-NOTICES.md](THIRD-PARTY-NOTICES.md). To report a security issue, see
[CONTRIBUTING](CONTRIBUTING.md#security-issue-notifications).

## Authors

- Jan Metzner
- Hubert Asamer
