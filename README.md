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

Programming a robot cell usually takes a separate tool and a hand-off for every step, and each new product variant
starts much of that work again. This Guidance brings these steps into one browser-based application on AWS, Physical
AI Production Designer, which an AI agent can drive and whose output is checked control code:

1. **Robo Dresser** mounts a tool on a robot and computes its collision hulls.
2. **Robo Fitter** lays out robots, fixtures and a floor plan into a cell, and checks which robot reaches which weld.
3. **Product & Task** holds the product and its welding task.
4. **Planning** assigns the welds to the robots and plans every motion, with learned policies, umarell's planners
   (U3 and umarell_jam), or an agentic planner on Amazon Bedrock. A Kiro agent runs the planners from a chat. Every
   plan is checked for joint and speed limits, collisions and timing before it is offered.
5. **Behaviour tree executor** replays the plan on the simulated cell in the 3D view.
6. **Control code**: plan2cell generates the PLC program (IEC 61131-3 Structured Text, for CODESYS) and the robots'
   KUKA KRL programs.

The sample data is the welding cell `welding_cell_2_small`: two KUKA KR 240 R2900 robots and 15 spot welds.

### What the installer deploys

The CloudFormation installer (`deployment/pai-pd-installer.yaml`) compiles the release in AWS CodeBuild and runs the
AWS CDK app in `deployment/` there. It creates two stacks: `factoryos` in your Region, and `factoryos-edge` in US East
(N. Virginia) for the Lambda@Edge function.

| Service | Use |
| --- | --- |
| Amazon CloudFront, Lambda@Edge | Serves the viewer; checks the Amazon Cognito session on every request before forwarding it. |
| Amazon Cognito | The deployment's user pool: managed login, multi-factor authentication, no self sign-up. |
| Amazon EC2 | One AWS Graviton instance (m7g.2xlarge, Ubuntu 24.04) in a private subnet, without a public IP address, running the shared workspace. Its Amazon EBS volume holds the designs, cells and plans. |
| Amazon VPC | A private subnet for the instance and a NAT gateway for its outbound traffic. |
| AWS CodeBuild | The installer: one project compiles the release for the instance, one deploys the stacks. |
| Amazon S3 | The release and its compiled bundle for the instance, and the access logs. |
| AWS Systems Manager | Administrative access (Session Manager, no SSH) and the release state. |
| Amazon CloudWatch | The instance's, the VPC's and the functions' logs. |
| Amazon Bedrock | Optional: the agentic planner's models. |

### Cost

_You are responsible for the cost of the AWS services used while running this Guidance. As of November 2026, the cost
for running this Guidance with the default settings in the US East (N. Virginia) Region is approximately $301.32 per
month for running one deployment around the clock for a team of up to 10 users._

_We recommend creating a [Budget](https://docs.aws.amazon.com/cost-management/latest/userguide/budgets-managing-costs.html)
through [AWS Cost Explorer](https://aws.amazon.com/aws-cost-management/aws-cost-explorer/) to help manage costs. Prices
are subject to change. For full details, refer to the pricing webpage for each AWS service used in this Guidance._

The instance is most of the cost and is billed for every hour it runs. Stopping it outside working hours (200 hours a
month) lowers its share from $238.27 to about $65.28.

### Sample Cost Table

The following table provides a sample cost breakdown for deploying this Guidance with the default parameters in the
US East (N. Virginia) Region for one month, at On-Demand list prices.

| AWS service  | Dimensions | Cost [USD] |
| ----------- | ------------ | ------------ |
| Amazon EC2 | 1 m7g.2xlarge instance, 730 hours | $ 238.27 |
| Amazon EBS | 200 GB gp3 volume | $ 16.00 |
| Amazon VPC: NAT gateway | 1 NAT gateway, 730 hours, 50 GB processed | $ 35.10 |
| Amazon VPC: public IPv4 address | 1 address, 730 hours | $ 3.65 |
| Amazon CloudFront | 50 GB out to the internet, 10 GB to the origin, 1,000,000 HTTPS requests | $ 5.45 |
| AWS Lambda@Edge | 1,000,000 invocations, 128 MB, 5 ms each | $ 0.63 |
| Amazon Cognito | 10 monthly active users (Essentials) | $ 0.00 |
| Amazon S3 | 2 GB Standard, about 22,000 requests | $ 0.06 |
| Amazon CloudWatch | 2.5 GB of logs ingested | $ 1.26 |
| Amazon EC2: data transfer out | 10 GB to the internet | $ 0.90 |
| AWS Systems Manager, AWS Lambda, AWS KMS, AWS CloudFormation | Within their free use | $ 0.00 |
| **Total** | | **$ 301.32** |
| AWS CodeBuild (per installation or update, not in the total) | 1 release build of about 17 minutes on Arm (arm1.xlarge), and the installer's build of about 25 minutes on general1.small | $ 1.15 |
| Amazon Bedrock (optional, not in the total) | 20 agentic planning sessions with 200,000 input and 20,000 output tokens each on Anthropic Claude Sonnet 5 | $ 13.20 |

## Prerequisites

### Operating System

You need a web browser and AWS CloudShell, which has the AWS CLI and git. The installer creates and prepares the
instance (Ubuntu 24.04 LTS, arm64); you do not prepare a host yourself.

### AWS account requirements

- Permission to create an AWS CloudFormation stack with IAM resources. The installer's CodeBuild project does the rest
  with its own role.
- 8 vCPUs of the Amazon EC2 quota **Running On-Demand Standard (A, C, D, H, I, M, R, T, Z) instances** in the Region.
- Optional, for the agentic planner: access to Anthropic models on Amazon Bedrock in the Region.
- A Kiro subscription for the planning agent ([kiro.dev](https://kiro.dev)): the viewer plans through its chat.

### aws cdk bootstrap

The Guidance uses the AWS CDK. The installer bootstraps it where needed, in your Region and in US East (N. Virginia);
you need not run `cdk bootstrap` yourself.

### Supported Regions

US East (N. Virginia) (`us-east-1`) is the default and tested Region. Any commercial Region with AWS Graviton m7g
instances, CloudFront VPC origins and Amazon Cognito managed login works.

## Deployment Steps

1. Open AWS CloudShell in the Region of the deployment, and clone the repository:
   ```sh
   git clone https://github.com/aws-solutions-library-samples/guidance-for-physical-ai-production-designer-on-aws
   ```
2. Create the installer stack, with the e-mail address of the first user. Amazon Cognito sends the sign-in URL and a
   temporary password to it.
   ```sh
   aws cloudformation create-stack --stack-name pai-pd-installer \
     --template-body file://guidance-for-physical-ai-production-designer-on-aws/deployment/pai-pd-installer.yaml \
     --capabilities CAPABILITY_IAM --on-failure DO_NOTHING \
     --parameters ParameterKey=AdminEmail,ParameterValue=you@example.com
   ```
   You can also create the stack in the [AWS CloudFormation console](https://console.aws.amazon.com/cloudformation/home)
   from the template `deployment/pai-pd-installer.yaml`.
3. Wait for the stack. It completes when the application is ready, after about 30 minutes: the installer compiles
   the release in AWS CodeBuild while it deploys the stacks, then installs it on the instance. The stack's
   `ReleaseBuildLogs` and `BuildLogs` outputs link to the logs.
   ```sh
   aws cloudformation describe-stacks --stack-name pai-pd-installer --query 'Stacks[0].[StackStatus,Outputs]'
   ```
4. Sign in at the URL from the invitation e-mail, choose a new password, and set up multi-factor authentication with
   an authenticator app. Then, in the viewer's settings, choose **Sign in to the agent** to sign the Kiro agent in.

To update later, update the installer stack with a new **Release**. The [operations guide](docs/operations.md) covers
users, updates and troubleshooting.

## Deployment Validation

1. The installer stack is `CREATE_COMPLETE`, and the application stack's `GateVerified` output is `true`:
   ```sh
   aws cloudformation describe-stacks --stack-name factoryos --query "Stacks[0].Outputs"
   ```
2. An anonymous request is denied. With the `Url` output in `URL`, this prints `401`:
   ```sh
   URL=$(aws cloudformation describe-stacks --stack-name factoryos \
     --query "Stacks[0].Outputs[?OutputKey=='Url'].OutputValue | [0]" --output text)
   curl -sS -o /dev/null -w '%{http_code}\n' "${URL%/}/api/v1/health"
   ```
3. Open the URL and sign in: the Physical AI Production Designer viewer loads.

## Running the Guidance

Sign in, and make sure the Kiro agent is signed in ([Deployment Steps](#deployment-steps), step 4). The viewer shows
the workflow as cards on the left, the 3D view in the centre and the open tool's panels on the right.

### 1. Product & Task: the part and its welds

Choose **Open Product & Task**, and in the **Catalogue** section choose `weld_box`. To make your own product, choose
**New**, set its geometry, add a **Spot welding** task with its points, and choose **Save**.

![Product & Task: a box product with labelled spot welds on its top face, and the panel with the task's table of weld positions](assets/images/walkthrough/product-and-task.jpg)

**Result:** the box with its spot welds `W01` to `W06`, and the task `spot_weld_1`.

### 2. Robo Dresser: mount the tool on the robot

Choose **Open Dresser**, pick the robot `kr240r2900` and the tool `sticktool`, and choose **Dress**.

**Result:** the dressed robot `kr240r2900__sticktool`, with no issues under **Diagnostics**.

### 3. Robo Fitter: lay out the cell

In the **Robo Fitter** card, pick the cell `welding_cell_2_small` and choose **Open**.

**Result:** the two robots, the fixture and the 15 weld points in the 3D view.

### 4. Agentic Planning: plan the motions

In the **Agentic Planning** card, pick the task program, leave the planner on `auto`, and choose **Plan Cell**. The
planner agent chooses a planner and plans the cell. In the chat you can ask for another planner, or add a constraint
in plain words.

![Agentic Planning: the planner chat beside a cell, with the agent's planner recommendation, its tool calls and the plan's progress](assets/images/walkthrough/agentic-planning.jpg)

**Result:** the agent names the planner, the plan, its cycle time and the checks it passed, after a few minutes.

### 5. Behaviour Tree: replay the plan

In the **Behaviour Tree** card, pick the behaviour tree of the plan and choose **Load & Play**.

![Behaviour Tree: two robots welding in the 3D view, with the playback progress and each robot's operations on a timeline](assets/images/walkthrough/behaviour-tree.jpg)

**Result:** the robots weld in the simulated cell, and the timeline shows each robot's operations.

### 6. Control Code Export: the PLC and robot programs

In the **Generate PLC + robot code** card, pick the behaviour tree and choose **Export PLC + robot code**.
**Download archive (.zip)** saves the build.

![Control Code Export: the export card with its options, next to the build's warnings and its file tree of KRL and PLC files](assets/images/walkthrough/control-code-export.jpg)

**Result:** a build with the PLC project (`plc/`), each robot's KRL program (`krl/robot1/`, `krl/robot2/`) and a
summary (`BUILD.md`).

## Next Steps

- Add your own robots, tools, fixtures and products, and lay out your own cells.
- Turn on the agentic planner: update the installer stack with **Amazon Bedrock** set to `true`.
- Explore the code generator in `source/plan2cell/README.md`.
- Operate the deployment with the [operations guide](docs/operations.md).

## Cleanup

1. Run the installer stack's `Uninstall` command (its output) in AWS CloudShell. It deletes the application's stacks
   with the instance, its data and the users.
2. When it has finished, delete the installer stack:
   ```sh
   aws cloudformation delete-stack --stack-name pai-pd-installer
   aws cloudformation wait stack-delete-complete --stack-name pai-pd-installer
   ```

The AWS CDK bootstrap stacks (`CDKToolkit`) stay; they are shared by every AWS CDK app in the account.

## FAQ, known issues, additional considerations, and limitations

### Security

- Anyone can reach the URL, and gets only the sign-in page until signed in.
- Every user shares the whole deployment, including its terminals, which are a shell on the instance with the
  instance's permissions. Invite only people you would give that access to.
- The instance accepts traffic only from CloudFront, and only requests that carry the deployment's origin secret.
- The instance's role includes the Systems Manager and CloudWatch agents' permissions, which cover the whole Region of
  the account. Install the Guidance in an account of its own if the account runs other workloads.
- After sign-out, a copied session stays valid for up to 60 minutes.
- The default `*.cloudfront.net` domain also accepts TLS 1.0 and 1.1; CloudFront reaches the instance over HTTP inside
  the VPC; Ubuntu packages are installed over HTTP, with signature checks.
- No AWS WAF web ACL and no Amazon Bedrock guardrail are configured. Treat cell and product files from others like
  code: their text can steer the agents.

### Additional considerations

- `source/cell-planner/runs/` holds pre-trained model checkpoints of the learned policies; they are data, not
  executables.
- Review, test and validate any generated control code on virtual controllers and under your own safety procedures
  before it runs on physical equipment.

For any feedback, questions, or suggestions, please use the issues tab under this repo.

## Revisions

- **Unreleased**: the first public version.

## Notices

*Customers are responsible for making their own independent assessment of the information in this Guidance. This Guidance: (a) is for informational purposes only, (b) represents AWS current product offerings and practices, which are subject to change without notice, and (c) does not create any commitments or assurances from AWS and its affiliates, suppliers or licensors. AWS products or services are provided “as is” without warranties, representations, or conditions of any kind, whether express or implied. AWS responsibilities and liabilities to its customers are controlled by AWS agreements, and this Guidance is not part of, nor does it modify, any agreement between AWS and its customers.*

References to third-party services or organizations in this Guidance do not imply an endorsement, sponsorship, or
affiliation between Amazon or AWS and the third party. Guidance from AWS is a technical starting point, and you can
customize your integration with third-party services when you deploy the architecture.

This Guidance is licensed under the MIT-0 License. See the [LICENSE](LICENSE) file. The third-party software it builds
on, and their licences: [THIRD-PARTY-NOTICES.md](THIRD-PARTY-NOTICES.md). To report a security issue, see
[CONTRIBUTING](CONTRIBUTING.md#security-issue-notifications).

## Authors

- Fabrizio Manfredi ([GitHub](https://github.com/f-manfred))
- Jan Metzner ([GitHub](https://github.com/jmetzner))
- Hubert Asamer ([GitHub](https://github.com/asamerh4))
