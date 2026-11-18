# shellcheck shell=bash
# deployment/installer/status.sh: what is installed and how the release on the instance is getting on.

# print_state <state JSON> [<log group>]: the release's phase and version; its error one part ("; ") a line (the
# failing step, its script's own error, a failed unit's journal error, the rollback); what a ready release provisioned
# without (`incomplete`, e.g. umarell_jam); after a failure, where its log is.
print_state() {
  local state="$1" lg="${2:-}"
  if [ -z "$state" ]; then say "${B}Release${N}  (the instance has not started yet)"; return; fi
  python3 -I -c '
import json, sys
try:
    j = json.loads(sys.argv[1])
except ValueError:
    print("(unreadable state: %s)" % sys.argv[1][:200]); sys.exit(0)
phase = j.get("phase") or "?"
head = "%s  %s" % (phase, j.get("version") or "")
if j.get("previous") and phase != "ready":
    head += "  (serving: %s)" % j["previous"]
print(head)
parts = [p.strip() for p in (j.get("error") or "").split("; ") if p.strip()]
for i, p in enumerate(parts):
    print(("  error: " if i == 0 else "         ") + p)
inc = [x for x in (j.get("incomplete") or []) if isinstance(x, str) and x.strip()]
if inc:
    print("  incomplete: %s (provisioned without it; the next update provisions it again)" % ", ".join(inc))
    if "umarell_jam" in inc:
        print("         umarell_jam: its jobs fall back to umarell_u3 (minutes, not seconds); why: the release log "
              "(WARNING: umarell_jam); an instance that cannot reach GitHub needs --umarell-jam-tarball")
if phase in ("failed", "rolled-back") and j.get("logStream"):
    print("  log:   %s%s (CloudWatch Logs)" % (sys.argv[2] + " " if sys.argv[2] else "", j["logStream"]))
' "$state" "$lg" | sed "1s/^/${B}Release${N}  /"
}

# log_tail [<lines>]: the release log's last lines (CloudWatch: <instance id>/release.log), a block of lines that
# repeats (a unit's journal while systemd restarts it, a health check's curl) shown once with its count, so the lines
# before it (the deploy's own error) stay in view. Each line at most 200 characters.
log_tail() {
  local lg iid n=${1:-20}; lg=$(stack_output LogGroup); iid=$(stack_output InstanceId)
  [ -n "$lg" ] && [ -n "$iid" ] || return 0
  awsr logs get-log-events --log-group-name "$lg" --log-stream-name "$iid/release.log" --limit $(( n * 10 )) \
    --no-start-from-head --query 'events[].message' --output json 2>/dev/null | collapse_log "$n" || true
}

# collapse_log <lines> < JSON list of log messages: the last <lines> lines, "  │ "-prefixed, repeats collapsed (a block
# of up to 8 lines that repeats right after itself: its last copy, then "… the N lines above: M times"). Lines are the
# same for this when they differ only in a journal line's date, host and pid, a restart counter, or curl's "after N ms".
collapse_log() {
  python3 -I -c '
import json, re, sys
n = int(sys.argv[1])
try:
    msgs = json.load(sys.stdin)
except ValueError:
    msgs = []
lines = []
for m in msgs if isinstance(msgs, list) else []:
    if isinstance(m, str):
        lines += [l.rstrip() for l in m.replace("\r", "").split("\n") if l.strip()]
subs = [(re.compile(r"^[A-Z][a-z]{2} [ 0-9]?[0-9] [0-9]{2}:[0-9]{2}:[0-9]{2} \S+ "), ""),
        (re.compile(r"\[[0-9]+\]"), "[]"), (re.compile(r"restart counter is at [0-9]+"), "restart counter"),
        (re.compile(r"after [0-9]+ ms"), "after ms")]
def norm(l):
    for r, t in subs:
        l = r.sub(t, l)
    return l
key = [norm(l) for l in lines]
out, i = [], 0
while i < len(lines):
    best_b, best_r = 1, 1
    for b in range(1, 9):
        if i + 2 * b > len(lines):
            break
        r = 1
        while i + (r + 1) * b <= len(lines) and key[i + r * b:i + (r + 1) * b] == key[i:i + b]:
            r += 1
        if r > 1 and b * r > best_b * best_r:
            best_b, best_r = b, r
    out += lines[i + (best_r - 1) * best_b:i + best_r * best_b]
    if best_r > 1:
        out.append("… %s: %d times" % ("the line above" if best_b == 1 else "the %d lines above" % best_b, best_r))
    i += best_b * best_r
for l in out[-n:]:
    print("  │ " + l[:200])
' "$1"
}

cmd_status() {
  local main edge; main=$(stack_status "$STACK" "$REGION"); edge=$(stack_status "$STACK-edge" "$EDGE_REGION")
  if [ "$main" = NONE ] && [ "$edge" = NONE ]; then say "Not installed ($STACK in $REGION)."; return 0; fi
  say "${B}Stacks${N}   $STACK: $main   $STACK-edge: $edge"
  say "${B}URL${N}      $(stack_output Url)"
  say "${B}Sign-in${N}  verified to deny anonymous access: $(stack_output GateVerified | sed 's/^$/no/')"
  local wanted state; wanted=$(get_param "$(param_wanted)"); state=$(get_param "$(param_state)")
  [ -z "$wanted" ] || say "${B}Wanted${N}   $(json 'j.get("version")' <<<"$wanted")  (requested $(json 'j.get("requestedAt")' <<<"$wanted"))"
  local lg; lg=$(stack_output LogGroup)
  print_state "$state" "$lg"
  say "${B}Users${N}"; list_users
  say "${B}Release log${N} (last lines)"; log_tail 20
  [ "$WAIT" = 1 ] || return 0
  local last="$state" phase inc
  while :; do
    phase=$( [ -n "$state" ] && json 'j.get("phase") or ""' <<<"$state" || echo "")
    case "$phase" in ready|failed|rolled-back) break ;; esac
    sleep "$POLL_SECONDS"
    state=$(get_param "$(param_state)")
    if [ "$state" != "$last" ]; then printf '%s  ' "$(date +%H:%M)"; print_state "$state" "$lg"; last="$state"; fi
  done
  case "$phase" in
    ready) ok "$(json 'j.get("version")' <<<"$state") is ready: $(stack_output Url)"
           inc=$(json '", ".join(x for x in (j.get("incomplete") or []) if isinstance(x, str))' <<<"$state" 2>/dev/null || true)
           [ -z "$inc" ] || warn "provisioning incomplete: $inc (the release serves; see incomplete above)" ;;
    *) log_tail 40; die "the release ended $phase" ;;
  esac
}
