#!/usr/bin/env bash
# Installs, updates and manages Physical AI Production Designer (Factory OS) in an AWS account.
# Run it in AWS CloudShell (or on any machine with the AWS CLI v2, Node.js 20+, python3 and credentials for the
# account), either from a git clone of the repository (git is needed then):
#
#   git clone <repository url> factoryos && cd factoryos
#   deployment/install.sh install --from-checkout --region us-east-1 --users you@example.com
#
# or from an unpacked export:
#
#   aws s3 cp s3://<bucket>/factoryos-<version>.zip /tmp/ && cd /tmp && unzip -q factoryos-<version>.zip
#   ./factoryos-<version>/deployment/install.sh s3://<bucket>/factoryos-<version>.zip --users you@example.com
#
# The same command installs into a fresh account or updates an existing installation (it finds the stacks).
# An update keeps the instance and its data (cells, catalogue, plans): the new release is built next to the running
# one, which serves until the switch; a release that fails its health check is switched back.
#
# Commands:
#   install.sh [install] <s3 uri of the export> [options]   install, or update to this export
#   install.sh [install] --from-checkout [options]          install, or update to the checkout this script is in
#   install.sh status [--wait]                              URL, versions, the release's progress, users
#   install.sh add-user <email>... | remove-user <email>...
#   install.sh uninstall [--snapshot] [--edge-only] [--confirm-stack <name>]
#                                                           delete everything (asks first)
#
# Options (all commands):
#   --region <region>        the stack's region. Default: $AWS_REGION (as CloudShell sets it) or your configuration;
#                            a region not given with --region is printed and asked about (--yes accepts it)
#   --stack-name <name>      the stack name (default: factoryos; also --stack). One name serves one region: a name
#                            installed in another region is refused (its sign-in at the edge is shared)
#   --yes                    do not ask before going on (destructive steps still need their own confirmation)
# Options (install):
#   --from-checkout          package the checkout this install.sh is in (the directory above deployment/) instead
#                            of an s3:// export, and upload it. A clean git checkout is `git archive` of HEAD; one
#                            with changes is packaged as it is on disk (ignored files left out), its version
#                            suffixed +dirty-<12 hex digits of the content's sha256>. The version is the release in
#                            source/VERSION.json or VERSION.json, else `git describe --tags --always`, else
#                            dev-<sha12>. The archive is staged in $FACTORYOS_WORK (default /tmp/factoryos-install).
#   --users <email,...>      users to invite (each gets an e-mail with a temporary password; MFA at first sign-in)
#   --instance-type <type>   Graviton instance type (default: m7g.2xlarge)
#   --volume-gb <n>          root volume size (default: 200)
#   --bedrock | --no-bedrock allow the instance to call Amazon Bedrock models (agentic planning); default off
#   --bedrock-models <ids>   the Bedrock inference profile ids (us.<provider>.<model>, ...) the instance may invoke,
#                            comma-separated, at most 20, no ARNs or wildcards (implies --bedrock; default: the
#                            agentic planner's models in the stack's region)
#   --kiro-secret <name>     Secrets Manager secret holding a Kiro API key for the planning agent ('' for none)
#                            An update keeps the installed value of each of these five options unless given again.
#   --umarell-tarball <s3>   umarell from a tarball (git archive --prefix=umarell/ of deploy/umarell.pin's commit)
#                            instead of GitHub
#   --umarell-jam-tarball <s3 | file>
#                            the plain umarell commit the umarell_jam planner builds on, from a tarball (git archive
#                            --prefix=umarell/ of deploy/umarell-jam.pin's commit; s3:// or a local file) instead of
#                            GitHub. Optional: without it (or when it is wrong) the release still installs, umarell_jam
#                            is marked incomplete and its jobs fall back to umarell_u3. An update keeps the installed
#                            one unless given again ('' drops it)
#   --build-project <name> --build-bucket <bucket>
#                            build the release in that AWS CodeBuild project (the CloudFormation installer's
#                            <installer stack>-build: Arm, Ubuntu 24.04), with its source staged under builds/ in that
#                            bucket, instead of on the instance; it starts before the stacks are deployed, and the
#                            instance then only deploys the bundle it built (deployment/CONTRACT.md "Release build in
#                            AWS CodeBuild"). Without them the instance builds the release itself
#   --force                  rebuild even if this version is installed already
#   --rotate-origin-secret   replace the origin secret (the x-origin-verify value the CloudFront gate sends and the
#                            instance requires) in both places, without a request refused for a wrong origin secret
#                            (about 8 minutes; also with the version installed already). The backend restarts twice
#                            (a few seconds each; the planner router with it): open terminals close, viewers
#                            reconnect, a running planning job is stopped; run it when nobody is working. Otherwise an
#                            update keeps it
#   --allow-downgrade        install an older version than the one installed
#   --allow-replace          allow an infrastructure change that replaces the instance, its volume, the source
#                            bucket or the user pool (their data is lost)
# Options (uninstall):
#   --confirm-stack <name>   confirm the deletion without a terminal: <name> must be the stack name (otherwise the
#                            stack name has to be typed)
#   --snapshot               keep an EBS snapshot of the instance's volume
#   --edge-only              delete only <stack>-edge in us-east-1 (a retry once CloudFront has released it)
set -euo pipefail

INSTALLER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export INSTALLER_DIR
# shellcheck source=installer/lib.sh
. "$INSTALLER_DIR/installer/lib.sh"
for part in preflight infra release users status uninstall origin; do
  # shellcheck disable=SC1090
  . "$INSTALLER_DIR/installer/$part.sh"
done

main "$@"
