#!/bin/bash
# T1 test of plan2cell's KRL on a KUKA VC controller in IO mode sim (run as root on the host,
# after `plan2cell cell krl` installed the KRL, p2c-kvc.py and p2c-headstub.py).
#
#   kvc-t1-test.sh [--netns krc1|krcN] [--api http://<kli of the netns>:8090] [--plan-id 16#F77E]
#                  [--bundle FILE] [--home folge125] [--prod folge11] [--timeout 900] [--no-home]
#
# The plan id: --plan-id, else plan_id of the bundle `plan2cell cell krl` installed (--bundle,
# default /var/lib/plan2cell/kuka-vc/bundles/robotN.json for krcN).
# 1. T1 only (refuses another mode); clears the bt flags, BT_PLANL := PLAN_ID (bt_sps latches it
#    only on a job request in EXT), confirms acknowledgement messages.
# 2. Home Folge: krl-run --select KRC:\R1\Folgen\<home> --on-halt exit.
# 3. Production Folge; p2c-headstub.py runs from step 2 on (DrivesOn $IN[1], PLAN_ID on $IN[33..48], auto-grant of every REQUEST,
#    T1 BCO unlatch): krl-run --select KRC:\R1\Folgen\<prod> --on-halt exit.
# 4. Summary: exit codes, the STEP values in the order they appeared, BT_ECODE / BT_WCODE,
#    PLAN_ID of the Folge, the messages. Logs in /var/lib/plan2cell/t1-test/<UTC>/.
# krl-run drives the robot only while it holds the (simulated) start key, in T1 (250 mm/s).
set -uo pipefail
NETNS=krc1 API="" PLAN="" BUNDLE="" HOMEF=folge125 PROD=folge11 TO=900 HOME_RUN=1
DIR=/var/lib/plan2cell/kuka-vc
while [ $# -gt 0 ]; do
  case $1 in
    --netns) NETNS=$2; shift ;;
    --api) API=$2; shift ;;
    --plan-id) PLAN=$2; shift ;;
    --bundle) BUNDLE=$2; shift ;;
    --home) HOMEF=$2; shift ;;
    --prod) PROD=$2; shift ;;
    --timeout) TO=$2; shift ;;
    --no-home) HOME_RUN=0 ;;
    *) echo "unknown argument $1" >&2; exit 2 ;;
  esac
  shift
done
case "$NETNS" in krc[1-9]) ;; *) echo "--netns must be krc1..krc9" >&2; exit 2 ;; esac
if [ -z "$PLAN" ]; then
  BUNDLE=${BUNDLE:-$DIR/bundles/robot${NETNS#krc}.json}
  PLAN=$(python3 -I -c 'import json,sys; print(json.load(open(sys.argv[1]))["manifest"]["plan_id"])' "$BUNDLE" 2>/dev/null) \
    || { echo "no plan_id in $BUNDLE: pass --plan-id or --bundle" >&2; exit 2; }
fi
# the host's krl-run / kss-msgs enter netns krc1 by themselves unless KRL_RUN_IN_NETNS=1; for a concurrent
# controller krcN (kvc-add-controller.sh) they run inside krcN instead
innet() { if [ "$NETNS" = krc1 ]; then "$@"; else ip netns exec "$NETNS" env KRL_RUN_IN_NETNS=1 "$@"; fi; }
if [ -z "$API" ]; then
  KLI=$(ip -n "$NETNS" -4 -o addr show dev kli scope global | awk '{print $4}' | cut -d/ -f1 | head -1)
  API=http://${KLI:-10.200.0.11}:8090
fi
TS=$(date -u +%Y%m%dT%H%M%SZ)
OUT=/var/lib/plan2cell/t1-test/$TS-$NETNS
mkdir -p "$OUT"
api() { ip netns exec "$NETNS" python3 /opt/kuka-ec2/krl/kssapi.py "$@"; }
show() { api VariableService/ShowVar "{\"identifier\":{\"name\":\"$1\"${2:+,\"data_path\":\"$2\"}}}" 2>/dev/null \
           | python3 -c 'import json,sys; print(json.load(sys.stdin).get("value"))' 2>/dev/null || echo "?"; }
setv() { api VariableService/SetVar "{\"identifier\":{\"name\":\"$1\"},\"value\":\"$2\"}" >/dev/null; }
log() { echo "$(date -u +%FT%TZ) kvc-t1-test $*" >> /var/lib/plan2cell/changes.log; }

MODE=$(show '$MODE_OP')
if [ "$MODE" != "#T1" ]; then echo "{\"ok\":false,\"error\":\"\$MODE_OP=$MODE, this test runs in T1 only\"}"; exit 3; fi
case "$PLAN" in 16#*) PLANINT=$(( 16#${PLAN#16#} )) ;; *[!0-9]*|"") echo "--plan-id: 16#hex or decimal" >&2; exit 2 ;; *) PLANINT=$(( 10#$PLAN )) ;; esac
log "start $NETNS $PROD plan $PLAN logs $OUT"
for v in BT_ABRT BT_RUN BT_ERR BT_FIN; do setv $v FALSE; done
setv BT_ECODE 0; setv BT_WCODE 0; setv BT_PLANL "$PLANINT"
innet kss-msgs --confirm > "$OUT/msgs-before.txt" 2>&1

# The stub runs for both Folgen: it holds DrivesOn ($IN[1] = $DRIVES_ON / $MOVE_ENABLE after machine-dat).
ip netns exec "$NETNS" python3 -I "$DIR/p2c-headstub.py" --api "$API" --plan-id "$PLAN" --duration "$TO" \
  --log "$OUT/stub.jsonl" --t1-bco-unlatch --drives > "$OUT/stub.out" 2>&1 &
STUB=$!
sleep 4
# DrivesOn raises acknowledgeable messages (M_1210): confirm them once the stub holds it.
innet kss-msgs --confirm >> "$OUT/msgs-before.txt" 2>&1
RC_HOME=-1
if [ "$HOME_RUN" = 1 ]; then
  innet krl-run --select "KRC:\\R1\\Folgen\\$HOMEF" --on-halt exit --timeout 300 > "$OUT/home.log" 2>&1
  RC_HOME=$?
fi
AX_HOME=$(show '$AXIS_ACT')
# bt_sps latches BT_ABRT when a running Folge stops (also the home Folge's end); in EXT bt_cell clears the
# flags per job, here the next Folge would return at its first op: clear them as before step 2.
for v in BT_ABRT BT_RUN BT_ERR BT_FIN; do setv $v FALSE; done

T0=$(date +%s)
innet krl-run --select "KRC:\\R1\\Folgen\\$PROD" --on-halt exit --timeout "$TO" > "$OUT/prod.log" 2>&1
RC_PROD=$?
T1=$(date +%s)
sleep 1
kill -TERM $STUB 2>/dev/null; wait $STUB 2>/dev/null
innet kss-msgs > "$OUT/msgs-after.txt" 2>&1
log "end $NETNS $PROD rc_home=$RC_HOME rc_prod=$RC_PROD"

python3 -I - "$OUT" "$RC_HOME" "$RC_PROD" "$((T1 - T0))" "$(show BT_ECODE)" "$(show BT_WCODE)" "$(show BT_PLANL)" \
  "$(show PLAN_ID /R1/${PROD^^})" "$(show '$AXIS_ACT')" "$AX_HOME" "$(show '$PRO_STATE1')" <<'PY'
import json, sys
out, rc_home, rc_prod, secs, ecode, wcode, planl, plan_id, axis, ax_home, st = sys.argv[1:12]
steps, grants, unlatch, errors = [], 0, 0, []
def lines(p):
    try:
        return open(p).readlines()
    except OSError:
        return []
for line in lines(out + "/stub.jsonl"):
    try:
        e = json.loads(line)
    except ValueError:
        continue
    if e.get("ev") == "io" and e["step"] and (not steps or steps[-1][0] != e["step"]):
        steps.append((e["step"], e["t"]))
    grants += e.get("ev") == "grant"
    unlatch += e.get("ev") == "t1_bco_unlatch"
    if e.get("ev") == "error":
        errors.append(e["error"])
msgs = [l.rstrip() for l in lines(out + "/msgs-after.txt") if l.strip()]
tail = [l.rstrip() for l in lines(out + "/prod.log")][-6:]
print(json.dumps({"ok": rc_prod == "0" and rc_home in ("0", "-1") and ecode == "0", "logs": out,
                  "rc_home": int(rc_home), "rc_prod": int(rc_prod), "prod_s": int(secs),
                  "steps": [s for s, _ in steps], "step_t": [t for _, t in steps], "grants": grants,
                  "t1_bco_unlatch": unlatch, "BT_ECODE": ecode, "BT_WCODE": wcode, "BT_PLANL": planl,
                  "PLAN_ID": plan_id, "axis_after_home": ax_home, "axis": axis, "pro_state1": st,
                  "messages": msgs, "stub_errors": errors[:5], "prod_tail": tail}, indent=1))
PY
