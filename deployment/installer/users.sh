# shellcheck shell=bash
# deployment/installer/users.sh: the user pool's users. Created only once the edge sign-in was verified to deny
# anonymous access (the stack's GateVerified output), with Cognito's own invitation (a temporary password by e-mail;
# TOTP MFA is set up at the first sign-in).

users_after_gate() {
  [ "$(stack_output GateVerified)" = true ] \
    || die "the stack does not report GateVerified=true: the sign-in at the edge was not verified, so no users are created"
}

pool_id() {
  local p; p=$(stack_output UserPoolId)
  [ -n "$p" ] || die "the stack has no UserPoolId output (is $STACK installed in $REGION?)"
  echo "$p"
}

valid_email() { [[ "$1" =~ ^[^@[:space:]]+@[^@[:space:]]+\.[^@[:space:]]+$ ]]; }

cmd_add_users() {
  users_after_gate
  local pool e out; pool=$(pool_id)
  step "Users"
  for e in "$@"; do
    [ -n "$e" ] || continue
    valid_email "$e" || die "not an e-mail address: $e"
    if out=$(awsr cognito-idp admin-create-user --user-pool-id "$pool" --username "$e" \
          --user-attributes Name=email,Value="$e" Name=email_verified,Value=true \
          --desired-delivery-mediums EMAIL --output text --query User.UserStatus 2>&1); then
      ok "$e invited ($out): an e-mail with a temporary password is on its way"
    elif grep -q UsernameExistsException <<<"$out"; then
      ok "$e exists already"
    else
      die "could not create $e: $out"
    fi
  done
}

cmd_remove_users() {
  local pool e out; pool=$(pool_id)
  step "Users"
  for e in "$@"; do
    confirm "Delete the user $e (they can no longer sign in)?"
    if out=$(awsr cognito-idp admin-delete-user --user-pool-id "$pool" --username "$e" 2>&1); then
      ok "$e deleted"
    elif grep -q UserNotFoundException <<<"$out"; then
      ok "$e does not exist"
    else
      die "could not delete $e: $out"
    fi
  done
}

list_users() {
  local pool; pool=$(stack_output UserPoolId); [ -n "$pool" ] || return 0
  awsr cognito-idp list-users --user-pool-id "$pool" \
    --query 'Users[].[Attributes[?Name==`email`]|[0].Value, UserStatus, Enabled]' --output text 2>/dev/null \
    | awk '{printf "  %-40s %-24s %s\n", $1, $2, ($3=="True"?"":"disabled")}'
}
