#!/usr/bin/env python3
"""p2c-kvc: plan2cell's helper on the KUKA VC host (iiQKA.OS2 / KSS 9.2 on arm64 Linux).

plan2cell (`plan2cell cell krl` / `verify`) copies this file and a KRL bundle to the host and runs
it as root, over SSM or ssh. It talks to the kernel API (gRPC 127.0.0.1:63004) inside the
controller's network namespace, through the host's /opt/kuka-ec2/krl/kssapi.py.

  p2c-kvc.py status --netns krc1 --instance NAME
  p2c-kvc.py deploy --netns krc1 --instance NAME --bundle FILE [--sps-base INSTANCE] [--force] [--dry-run]
  p2c-kvc.py verify --netns krc1 --instance NAME --bundle FILE
  p2c-kvc.py machine-dat --instance NAME [--drives-off] [--move-enable '$IN[8]'] [--dry-run] [--force]   (host files; controller stopped)
  p2c-kvc.py program --netns krc1 show|cancel --name BT_CELL|select --module 'KRC:\\R1\\Program\\BT_CELL'
  p2c-kvc.py braketest-need --netns krc1 [--status FILE]      (FILE: 'kuka-braketest status --json' of the host)

Every command prints one JSON object (the last line of stdout) and keeps the output short (SSM
returns at most 24000 characters).

deploy: checks the bundle's files against the bt2krl manifest (fnv1a64), then makes the controller
match it, idempotently:
  * KRC/R1/{Folgen,Program,System}/* of the bundle (not bt_config.dat) are uploaded where they differ;
  * KRC:\\R1\\System\\$config.dat gets bt_config.dat between ';BT2KRL BEGIN' / ';BT2KRL END' at the end
    of the fold USER GLOBALS;
  * KRC:\\R1\\System\\sps.sub gets BT_SPS() between the markers in the fold USER PLC. With --sps-base
    the file is first reset to that instance's sps.sub on disk (e.g. the stock iiQKA.OS2_KSS_9.2.0, to
    drop the VW port's submit loop); the stock 'RUN /R1/CELL()' of AUTOEXT INIT becomes BT_CELL.
Changed files are saved to /var/lib/plan2cell/krl-backup/<UTC>/ first and every change is logged
in /var/lib/plan2cell/changes.log. The robot interpreter is cancelled, the submit is stopped and
cancelled while system files change, and started again afterwards; acknowledgement messages are
confirmed. Exit codes: 0 ok, 3 precondition (wrong instance, hash mismatch, program running),
4 compile errors, 1 other failure.

machine-dat: remaps KRC/STEU/Mada/$machine.dat of the instance to the head PLC's telegram (bt2krl
INSTALL.md): $DRIVES_ON=$IN[1] (DrivesOn), $EXT_START=$IN[2] (ProgStart), $CONF_MESS=$IN[6] (Reset),
$MOVE_ENABLE=$IN[1] (with --drives-off also $DRIVES_OFF=$IN[1]); every other SIGNAL on $IN[1..256]
(the DIO256 telegram the head now writes: the VW port's $IN[10/11/20/21], brake/mastering test requests
$IN[249/250]) becomes $IN[1026] (always FALSE). Each changed line keeps its comment and gets ';P2C was <old>'. The first
original is kept in /var/lib/plan2cell/kuka-vc/machine-dat/<instance>.orig. Takes effect at the next
controller start; refused (exit 3) while a controller runs the instance, unless --force.

program: the robot interpreter for kvc-braketest.sh. show: $MODE_OP, $PRO_STATE1, $PRO_NAME1[], $BRAKETEST_REQ_INT and
the brake-test monitoring time left. cancel --name N: deselects the selected program only when it is N (StopInterpreter
while #P_ACTIVE, then CancelInterpreter, as KUKA's krl-run / kuka-braketest do) and waits for #P_FREE; free already: ok;
another program: refused (exit 3). select --module M: SyncSelectService/Select of M while the interpreter is free (the
same as the AUTOEXT INIT 'RUN /R1/BT_CELL()' of sps.sub, which runs only at a submit start); M selected already: ok;
another program selected: refused (exit 3). Exit 1: the kernel API failed or the state did not come.

braketest-need: whether the current controller start still needs KUKA's brake test: M_27002 / M_27003 / M_27004
pending or $BRAKETEST_REQ_INT TRUE, or (the request not readable) no 'passed' in kuka-braketest's status for this start.
$BRAKETEST_REQ_INT FALSE without those messages: not needed (kuka-braketest run would say not-required).

verify: the deployed files read back with the manifest's hashes, the markers carry the plan, no
compile errors, the submit runs, PLAN_ID of the production Folge, the active messages.
"""
import argparse
import base64
import datetime
import json
import os
import re
import sys
import time

KRL_DIR = "/opt/kuka-ec2/krl"
INSTANCES = "/opt/kuka/instances"
STATE_DIR = "/var/lib/plan2cell"
LOG = STATE_DIR + "/changes.log"
BEGIN, END = ";BT2KRL BEGIN", ";BT2KRL END"
CONFIG = "KRC:\\R1\\System\\$config.dat"
SPS = "KRC:\\R1\\System\\sps.sub"
SYSTEM_FILES = (CONFIG, SPS)


# ---------------------------------------------------------------- pure functions (unit tested)

def fnv1a64(b):
    h = 0xCBF29CE484222325
    for x in b:
        h = ((h ^ x) * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return "%016x" % h


def eol_of(text):
    return "\r\n" if "\r\n" in text else "\n"


def strip_block(text):
    """text without the ';BT2KRL BEGIN' .. ';BT2KRL END' lines (both included)."""
    out, inside = [], False
    for line in text.splitlines(keepends=True):
        s = line.strip()
        if s.startswith(BEGIN):
            inside = True
            continue
        if inside:
            if s.startswith(END):
                inside = False
            continue
        out.append(line)
    if inside:
        raise ValueError("unterminated %s block" % BEGIN)
    return "".join(out)


def _lines(text, eol):
    """The lines of `text` with `eol` endings (a trailing line break adds no empty line)."""
    parts = text.replace("\r\n", "\n").split("\n")
    while parts and parts[-1] == "":
        parts.pop()
    return [l + eol for l in parts]


def patch_config(current, fragment, tag):
    """$config.dat with `fragment` (bt_config.dat) between the markers before ';ENDFOLD (USER GLOBALS)'."""
    eol = eol_of(current)
    base = strip_block(current)
    lines = base.splitlines(keepends=True)
    at = [i for i, l in enumerate(lines) if l.strip().upper().startswith(";ENDFOLD (USER GLOBALS)")]
    if not at:
        raise ValueError("$config.dat has no ';ENDFOLD (USER GLOBALS)'")
    block = [BEGIN + " " + tag + eol] + _lines(fragment, eol) + [END + eol]
    i = at[-1]
    return "".join(lines[:i] + block + lines[i:])


def patch_sps(base, calls, tag, ext_program=None):
    """sps.sub with `calls` between the markers right after ';FOLD USER PLC' (and its comment line)."""
    eol = eol_of(base)
    text = strip_block(base)
    if ext_program:
        text = re.sub(r'"RUN /R1/CELL\(\)"', '"RUN /R1/%s()"' % ext_program, text)
    lines = text.splitlines(keepends=True)
    at = [i for i, l in enumerate(lines) if l.strip().upper().startswith(";FOLD USER PLC")]
    if not at:
        raise ValueError("sps.sub has no ';FOLD USER PLC'")
    i = at[0] + 1
    if i < len(lines) and lines[i].strip().startswith(";") and "modification" in lines[i].lower():
        i += 1
    ind = re.match(r"\s*", lines[at[0]]).group(0)
    block = [ind + BEGIN + " " + tag + eol] + [ind + c + eol for c in calls] + [ind + END + eol]
    return "".join(lines[:i] + block + lines[i:])


BRAKETEST_KEYS = ("M_27002", "M_27003", "M_27004")   # brake test overdue (QUITT, stop) / required (STATE)
BRAKETEST_OK = ("passed", "not-required")


def pro_name(v):
    """$PRO_NAME1[] as the kernel API returns it ('"BT_CELL"', padded) -> BT_CELL."""
    return (v or "").strip().strip('"').strip()


def module_name(module):
    """KRC:\\R1\\Program\\bt_cell(.src) -> BT_CELL (what $PRO_NAME1[] shows once it is selected)."""
    m = module.replace("/", "\\").rstrip("\\").rsplit("\\", 1)[-1]
    return (m[:-4] if m.lower().endswith(".src") else m).upper()


def braketest_need(status, req, messages):
    """(need, why) for the current controller start. status: the JSON of 'kuka-braketest status --json' (dict or None),
    req: $BRAKETEST_REQ_INT ('TRUE' / 'FALSE' / anything else = unknown), messages: p2c-kvc.py's message list."""
    pending = sorted({str(m.get("key")) for m in messages or [] if isinstance(m, dict) and m.get("key") in BRAKETEST_KEYS})
    st = status if isinstance(status, dict) else {}
    passed = st.get("current") is True and st.get("result") in BRAKETEST_OK
    seen = ("%s for this controller start" % st.get("result") if st.get("current") is True
            else "no result for this controller start" if st else "no kuka-braketest status")
    if pending:
        return True, "%s pending (%s)" % (", ".join(pending), seen)
    if req == "TRUE":
        return True, "$BRAKETEST_REQ_INT TRUE (%s)" % seen
    if req == "FALSE":
        return False, "the controller requests no brake test ($BRAKETEST_REQ_INT FALSE; %s)" % seen
    if passed:
        return False, "%s ($BRAKETEST_REQ_INT %s)" % (st.get("summary") or seen, req)
    return True, "%s, $BRAKETEST_REQ_INT not readable (%s)" % (seen, req)


def cancel_plan(pro_state, name, want):
    """What 'program cancel --name want' does with the interpreter: ('free', why) | ('cancel', why) | ('refuse', why)."""
    if pro_state == "#P_FREE":
        return "free", "robot interpreter free already"
    if not pro_state or str(pro_state).startswith("ERROR"):
        return "refuse", "$PRO_STATE1 not readable (%s)" % pro_state
    if pro_name(name).upper() != want.upper():
        return "refuse", "%s %s is selected, not %s: left alone" % (pro_name(name) or "?", pro_state, want)
    return "cancel", "%s %s" % (pro_name(name), pro_state)


def krc_path(rel):
    """KRC/R1/Folgen/folge11.src -> KRC:\\R1\\Folgen\\folge11.src"""
    if not rel.startswith("KRC/"):
        raise ValueError("not below KRC/: " + rel)
    return "KRC:\\" + rel[4:].replace("/", "\\")


def bundle_targets(files):
    """The bundle files that go to the controller as they are (bt_config.dat is merged instead)."""
    return sorted(
        (p for p in files
         if p.startswith("KRC/R1/") and p.lower().endswith((".src", ".dat", ".sub"))
         and not p.lower().endswith("/bt_config.dat")),
        key=lambda p: (p.rsplit("/", 1)[0], not p.lower().endswith(".dat"), p.lower()))


def check_bundle(bundle):
    """[(path, problem)] of the bundle against its manifest."""
    man = bundle["manifest"]
    files = bundle["files"]
    bad = []
    for f in man.get("files", []):
        if f["path"] not in files:
            bad.append((f["path"], "missing"))
        elif fnv1a64(files[f["path"]]) != f["fnv1a64"]:
            bad.append((f["path"], "fnv1a64 %s, manifest %s" % (fnv1a64(files[f["path"]]), f["fnv1a64"])))
    listed = {f["path"] for f in man.get("files", [])}
    for p in files:
        if p not in listed:
            bad.append((p, "not in the manifest"))
    return bad


def tag_of(bundle):
    m = bundle["manifest"]
    return "%s %s" % (m.get("robot", {}).get("ns", "?"), m.get("plan_id", "?"))


def plan_int(plan_id):
    """'16#8024' -> 32804"""
    s = str(plan_id).strip()
    return int(s[3:], 16) if s.startswith("16#") else int(s)


def load_bundle(path):
    with open(path) as f:
        b = json.load(f)
    if b.get("schema") != "plan2cell/krl-bundle@1":
        raise ValueError("%s: not a plan2cell/krl-bundle@1" % path)
    b["files"] = {p: base64.b64decode(c) for p, c in b["files"].items()}
    return b


# ---------------------------------------------------------------- controller access

def enter_netns(netns):
    if os.environ.get("P2C_KVC_NETNS") == netns:
        return
    if os.geteuid() != 0:
        sys.exit(json.dumps({"ok": False, "error": "run as root"}))
    os.environ["P2C_KVC_NETNS"] = netns
    os.environ["KRL_RUN_IN_NETNS"] = "1"   # the host's tools must not re-exec into krc1 themselves
    os.execvp("ip", ["ip", "netns", "exec", netns, sys.executable, "-I", os.path.abspath(__file__)] + sys.argv[1:])


class Kvc:
    def __init__(self):
        sys.path.insert(0, KRL_DIR)
        import grpc  # noqa: F401
        from kssapi import KssApi
        self.grpc = sys.modules["grpc"]
        self.api = KssApi(timeout=120)

    def var(self, name, data_path=None):
        ident = {"name": name}
        if data_path:
            ident["data_path"] = data_path
        try:
            return self.api.call("VariableService/ShowVar", {"identifier": ident}).get("value")
        except self.grpc.RpcError as e:
            return "ERROR " + (e.details() or e.code().name)

    def set_var(self, name, value):
        self.api.call("VariableService/SetVar", {"identifier": {"name": name}, "value": str(value)})

    def read(self, path):
        try:
            r = self.api.call("SyncFileService/CopyFile2Mem", {"source_path": path})
        except self.grpc.RpcError:
            return None
        return base64.b64decode(r.get("content", ""))

    def upload(self, path, content):
        self.api.call("SyncFileService/CopyMem2File", {
            "destination_path": path, "content": base64.b64encode(content).decode(),
            "options": {"overwrite_existing": True, "overwrite_read_only": True, "no_krl_analysis": False}})

    def mkdir(self, path):
        try:
            self.api.call("SyncFileService/CreateItem", {"full_path": path, "item_type": "ITEM_TYPE_DIR"})
        except self.grpc.RpcError as e:
            if (e.details() or "") != "ItemWithSameName":
                raise

    def errors(self, path):
        try:
            r = self.api.call("SyncCodeErrorService/ListModuleErrors", {"full_path": path})
        except self.grpc.RpcError as e:
            if (e.details() or "") == "BadErrorList":   # the kernel's "no error list"
                return []
            return [{"module": path, "error": "ListModuleErrors failed: %s" % e.details()}]
        return [{"module": e.get("module") or path, "line": e.get("error_line", 0),
                 "number": e.get("error_number", 0), "param": e.get("error_parameter", "")}
                for e in r.get("errors", [])]

    def interp(self, method, which):
        try:
            self.api.call("SyncSelectService/%s" % method, {"interpreter": which})
            return None
        except self.grpc.RpcError as e:
            return e.details() or e.code().name

    def wait_var(self, name, want, timeout):
        end = time.time() + timeout
        while time.time() < end:
            if self.var(name) in want:
                return True
            time.sleep(0.2)
        return False

    def messages(self, confirm=False):
        import importlib.util
        spec = importlib.util.spec_from_file_location("krlrun", os.path.join(KRL_DIR, "krl-run.py"))
        kr = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(kr)
        m = kr.Messages(self.api)
        time.sleep(1.5)
        m.poll(quiet=True)
        confirmed = []
        if confirm:
            for a in list(m.active.values()):
                if a.get("message_type") == "MESSAGE_TYPE_QUITT":
                    try:
                        self.api.call("SyncMessageService/ConfirmMessage",
                                      {"message_handle": a["message_handle"], "client_id": ""})
                        confirmed.append(a.get("message_db_key"))
                    except Exception:  # noqa: BLE001
                        pass
            time.sleep(1)
            m.poll(quiet=True)
        out = [{"key": a.get("message_db_key"), "type": a.get("message_type", "")[13:],
                "stop": bool(a.get("movement_implication", {}).get("robot_stop")),
                "text": " | ".join(p.get("value", "") for p in a.get("parameters", []))[:120]}
               for a in m.active.values()]
        try:
            m.stream.cancel()
        except Exception:  # noqa: BLE001
            pass
        return out, confirmed


def instance_file(netns=None):
    """The instance file of the controller in `netns`: krc1 is KUKA's kss-runtime (the kuka-switch
    instance in /opt/kuka/active-instance.ini); krcN (N > 1) is a concurrent controller added by
    kvc-add-controller.sh, whose instance is in /etc/kuka-ec2/krcN/active-instance.ini."""
    netns = netns or os.environ.get("P2C_KVC_NETNS") or "krc1"
    own = "/etc/kuka-ec2/%s/active-instance.ini" % netns
    if netns != "krc1" and os.path.exists(own):
        return own
    return "/opt/kuka/active-instance.ini"


def active_instance(netns=None):
    try:
        for line in open(instance_file(netns)):
            k, _, v = line.partition("=")
            if k.strip() == "KRC_INSTANCE_PATH":
                return os.path.basename(v.strip().rstrip("/"))
    except OSError:
        pass
    return None


def log_change(msg):
    os.makedirs(STATE_DIR, exist_ok=True)
    with open(LOG, "a") as f:
        f.write("%s p2c-kvc %s\n" % (datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"), msg))


MAX_MESSAGES = 40


def emit(obj, code=0):
    if isinstance(obj.get("messages"), list) and len(obj["messages"]) > MAX_MESSAGES:
        obj = dict(obj, messages=obj["messages"][:MAX_MESSAGES], messages_total=len(obj["messages"]))
    print(json.dumps(obj, separators=(",", ":")))
    sys.stdout.flush()
    os._exit(code)


def state(k):
    return {n: k.var(v) for n, v in (("mode_op", "$MODE_OP"), ("pro_state0", "$PRO_STATE0"),
                                      ("pro_state1", "$PRO_STATE1"), ("pro_name1", "$PRO_NAME1[]"))}


# ---------------------------------------------------------------- commands

def cmd_status(a):
    k = Kvc()
    msgs, _ = k.messages()
    st = state(k)
    st.update({"ok": True, "active_instance": active_instance(), "instance_ok": active_instance() == a.instance,
               "messages": msgs})
    emit(st)


def krl_key(b):
    """What KSS keeps of a KRL file, for comparing an upload with its read-back: KSS stores LF line
    endings and gives a module's .dat the &COMMENT of its .src, so both are left out."""
    if b is None:
        return None
    lines = b.replace(b"\r\n", b"\n").split(b"\n")
    return b"\n".join(l for l in lines if not l.startswith(b"&COMMENT")).rstrip(b"\n")


def desired(k, bundle, sps_base):
    """[(krc path, wanted bytes, current bytes or None)] for every file deploy manages."""
    tag = tag_of(bundle)
    out = []
    for rel in bundle_targets(bundle["files"]):
        p = krc_path(rel)
        out.append((p, bundle["files"][rel], k.read(p)))
    frag = bundle["files"].get("KRC/R1/System/bt_config.dat")
    cur = k.read(CONFIG)
    if cur is None:
        raise RuntimeError("cannot read " + CONFIG)
    if frag is not None:
        text = cur.decode("latin-1")
        want = patch_config(text, frag.decode("latin-1").replace("\r\n", "\n"), tag).encode("latin-1")
        out.append((CONFIG, want, cur))
    cur = k.read(SPS)
    if cur is None:
        raise RuntimeError("cannot read " + SPS)
    base = cur
    if sps_base:
        p = os.path.join(INSTANCES, sps_base, "ROBOTER/KRC/R1/System/sps.sub")
        with open(p, "rb") as f:
            base = f.read()
    has_sps = any(r.lower().endswith("/bt_sps.src") for r in bundle["files"])
    calls = ["BT_SPS()"] if has_sps else []
    want = patch_sps(base.decode("latin-1"), calls, tag,
                     ext_program="BT_CELL" if sps_base else None).encode("latin-1")
    out.append((SPS, want, cur))
    return out


def cmd_deploy(a):
    try:
        bundle = load_bundle(a.bundle)
    except (OSError, ValueError) as e:
        emit({"ok": False, "error": str(e)}, 3)
    bad = check_bundle(bundle)
    if bad:
        emit({"ok": False, "error": "bundle does not match its manifest", "files": bad}, 3)
    inst = active_instance()
    if inst != a.instance:
        emit({"ok": False, "error": "active instance %s, want %s" % (inst, a.instance)}, 3)
    k = Kvc()
    st = state(k)
    if st["pro_state1"] == "#P_ACTIVE" and not a.force:
        emit({"ok": False, "error": "a robot program is running (%s); stop it or give --force" % st["pro_name1"],
              "state": st}, 3)
    plan = desired(k, bundle, a.sps_base)
    changed = [(p, w, c) for p, w, c in plan if krl_key(w) != krl_key(c)]
    res = {"ok": True, "tag": tag_of(bundle), "state": st, "changed": [p for p, _, _ in changed],
           "unchanged": len(plan) - len(changed), "dry_run": a.dry_run}
    if a.dry_run:
        emit(res)
    notes = []
    if changed:
        ts = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        bdir = os.path.join(STATE_DIR, "krl-backup", ts)
        os.makedirs(bdir, exist_ok=True)
        for p, _, c in changed:
            if c is not None:
                with open(os.path.join(bdir, p.replace("KRC:\\", "").replace("\\", "_")), "wb") as f:
                    f.write(c)
        res["backup"] = bdir
        log_change("deploy %s on %s: backup of %d file(s) in %s" % (res["tag"], a.instance,
                                                                     len([c for c in changed if c[2] is not None]), bdir))
        if st["pro_state1"] not in ("#P_FREE", None):
            e = k.interp("CancelInterpreter", "INTERPRETER_ROBOT")
            notes.append("robot interpreter cancelled" + (" (%s)" % e if e else ""))
            time.sleep(0.5)
        system = any(p in SYSTEM_FILES or p.lower().endswith("bt_sps.src") for p, _, _ in changed)
        if system and k.var("$PRO_STATE0") != "#P_FREE":
            k.interp("StopInterpreter", "INTERPRETER_SYSTEM_SUBMIT")
            time.sleep(0.5)
            k.interp("CancelInterpreter", "INTERPRETER_SYSTEM_SUBMIT")
            if not k.wait_var("$PRO_STATE0", ("#P_FREE",), 10):
                emit(dict(res, ok=False, error="submit did not stop ($PRO_STATE0=%s)" % k.var("$PRO_STATE0")), 1)
            notes.append("submit stopped")
        order = sorted(changed, key=lambda x: (x[0] != CONFIG, x[0] == SPS))
        for p, w, _ in order:
            k.mkdir(p.rsplit("\\", 1)[0])
            k.upload(p, w)
            log_change("deploy %s: uploaded %s (fnv1a64 %s)" % (res["tag"], p, fnv1a64(w)))
        try:
            k.api.call("SyncFileService/AnalyseAll", {})
        except k.grpc.RpcError as e:
            notes.append("AnalyseAll: %s" % e.details())
    errs = []
    for p, _, _ in plan:
        errs.extend(k.errors(p))
    res["errors"] = errs[:40]
    if k.var("$PRO_STATE0") != "#P_ACTIVE":
        e = k.interp("StartInterpreter", "INTERPRETER_SYSTEM_SUBMIT")
        ok = k.wait_var("$PRO_STATE0", ("#P_ACTIVE",), 10)
        notes.append("submit started" if ok else "submit start failed: %s ($PRO_STATE0=%s)" % (e, k.var("$PRO_STATE0")))
        if changed:
            log_change("deploy %s: submit %s" % (res["tag"], "started" if ok else "start FAILED"))
    msgs, confirmed = k.messages(confirm=True)
    res.update({"notes": notes, "confirmed": confirmed, "messages": msgs, "pro_state0": k.var("$PRO_STATE0")})
    if errs:
        emit(dict(res, ok=False, error="%d compile error(s)" % len(errs)), 4)
    if res["pro_state0"] != "#P_ACTIVE":
        emit(dict(res, ok=False, error="submit not running ($PRO_STATE0=%s)" % res["pro_state0"]), 1)
    emit(res)


def cmd_verify(a):
    try:
        bundle = load_bundle(a.bundle)
    except (OSError, ValueError) as e:
        emit({"ok": False, "error": str(e)}, 3)
    k = Kvc()
    tag = tag_of(bundle)
    checks = []

    def check(expr, expect, value):
        checks.append({"expr": expr, "expect": str(expect), "value": None if value is None else str(value),
                       "ok": value is not None and str(value) == str(expect)})

    check("active_instance", a.instance, active_instance())
    bad = []
    for rel in bundle_targets(bundle["files"]):
        got = k.read(krc_path(rel))
        if krl_key(got) != krl_key(bundle["files"][rel]):
            bad.append(rel)
    check("files_match_manifest", "all", "all" if not bad else "differ: " + ", ".join(bad))
    cfg = (k.read(CONFIG) or b"").decode("latin-1")
    sps = (k.read(SPS) or b"").decode("latin-1")
    check("$config.dat marker", tag, tag if (BEGIN + " " + tag) in cfg else None)
    check("sps.sub marker", tag, tag if (BEGIN + " " + tag) in sps else None)
    errs = []
    for p in [krc_path(r) for r in bundle_targets(bundle["files"])] + list(SYSTEM_FILES):
        errs.extend(k.errors(p))
    check("compile_errors", 0, len(errs))
    check("$PRO_STATE0", "#P_ACTIVE", k.var("$PRO_STATE0"))
    prod = bundle["manifest"].get("check", {}).get("production")  # KRC/R1/Folgen/folge11
    if prod:
        mod = "/" + "/".join(prod.split("/")[1:]).upper()          # /R1/FOLGEN/FOLGE11
        name = prod.rsplit("/", 1)[-1].upper()
        want = plan_int(bundle["manifest"]["plan_id"])
        got = None
        for dp in ("/R1/" + name, mod):
            v = k.var("PLAN_ID", dp)
            if v is not None and not str(v).startswith("ERROR"):
                got = v
                break
        check("%s PLAN_ID" % name, want, got)
    msgs, _ = k.messages()
    state_msgs = [m for m in msgs if m["type"] == "STATE" or m["stop"]]
    check("state_or_stop_messages", 0, len(state_msgs))
    emit({"ok": all(c["ok"] for c in checks), "tag": tag, "checks": checks, "errors": errs[:20],
          "messages": msgs, "state": state(k)})


def program_state(k):
    st = {n: k.var(v) for n, v in (("mode_op", "$MODE_OP"), ("pro_state1", "$PRO_STATE1"),
                                    ("pro_name1", "$PRO_NAME1[]"), ("braketest_req", "$BRAKETEST_REQ_INT"),
                                    ("braketest_remaining_h", "GET_BRAKETEST_TIME(#BT_REMAINING_MON_TIME)"))}
    st["pro_name1"] = pro_name(st["pro_name1"])
    return st


def cmd_program(a):
    k = Kvc()
    st = program_state(k)
    res = dict(st, ok=True, action=a.action)
    if a.action == "show":
        emit(res)
    if a.action == "cancel":
        what, why = cancel_plan(st["pro_state1"], st["pro_name1"], a.name)
        res["note"] = why
        if what == "free":
            emit(res)
        if what == "refuse":
            emit(dict(res, ok=False, error=why), 3)
        if st["pro_state1"] == "#P_ACTIVE":
            e = k.interp("StopInterpreter", "INTERPRETER_ROBOT")
            res["stop"] = e or "ok"
            k.wait_var("$PRO_STATE1", ("#P_STOP", "#P_FREE", "#P_RESET", "#P_END"), 5)
        e = k.interp("CancelInterpreter", "INTERPRETER_ROBOT")
        res["cancel"] = e or "ok"
        ok = k.wait_var("$PRO_STATE1", ("#P_FREE",), a.timeout)
        log_change("program cancel %s (%s): %s" % (a.name, why, "free" if ok else "NOT free: %s" % k.var("$PRO_STATE1")))
        res.update(program_state(k))
        if not ok:
            emit(dict(res, ok=False, error="robot interpreter not free after CancelInterpreter (%s)" % (e or "no error")), 1)
        emit(res)
    want = module_name(a.module)
    if st["pro_name1"].upper() == want and st["pro_state1"] not in ("#P_FREE", None):
        emit(dict(res, note="%s selected already (%s)" % (want, st["pro_state1"])))
    if st["pro_state1"] != "#P_FREE":
        emit(dict(res, ok=False, error="robot interpreter busy: %s %s, not %s: left alone"
                  % (st["pro_name1"] or "?", st["pro_state1"], want)), 3)
    try:
        k.api.call("SyncSelectService/Select", {"module_name": a.module, "force_select": True})
        res["select"] = "ok"
    except k.grpc.RpcError as e:
        res["select"] = e.details() or e.code().name
    end = time.time() + a.timeout
    while time.time() < end and pro_name(k.var("$PRO_NAME1[]")).upper() != want:
        time.sleep(0.2)
    res.update(program_state(k))
    ok = res["pro_name1"].upper() == want and res["pro_state1"] not in ("#P_FREE", None)
    log_change("program select %s: %s" % (a.module, "%s %s" % (res["pro_name1"], res["pro_state1"]) if ok
                                          else "FAILED (%s, %s)" % (res["select"], res["pro_state1"])))
    if not ok:
        emit(dict(res, ok=False, error="%s not selected (Select: %s; $PRO_NAME1[] %s, $PRO_STATE1 %s)"
                  % (want, res["select"], res["pro_name1"] or "?", res["pro_state1"])), 1)
    emit(res)


def cmd_braketest_need(a):
    status = None
    if a.status:
        try:
            with open(a.status) as f:
                status = json.load(f)
        except (OSError, ValueError):
            status = None
    k = Kvc()
    msgs, _ = k.messages()
    st = program_state(k)
    if str(st["mode_op"]).startswith("ERROR"):
        emit(dict(st, ok=False, error="kernel API: %s" % st["mode_op"]), 1)
    need, why = braketest_need(status, st["braketest_req"], msgs)
    st.update(ok=True, need=need, why=why, pending=sorted({m["key"] for m in msgs if m["key"] in BRAKETEST_KEYS}),
              stop=[m["key"] for m in msgs if m["stop"]],
              status={x: (status or {}).get(x) for x in ("result", "current", "summary")})
    emit(st)


# ---------------------------------------------------------------- $machine.dat (stage 2)

MACHINE_DAT_MAP = {"$DRIVES_ON": "$IN[1]", "$EXT_START": "$IN[2]", "$CONF_MESS": "$IN[6]",
                   "$MOVE_ENABLE": "$IN[1]"}
TELEGRAM_INPUTS = range(1, 257)       # DIO256 slot 2: $IN[1..256] in profinet mode
IN_FALSE = "$IN[1026]"                # KSS: always FALSE (the file's own default for unused inputs)
TELEGRAM_OUTPUTS = range(1, 257)      # DIO256 slot 2: $OUT[1..256], written by bt2krl's signals
OUT_SPARE = 4081                      # first output tried for system signals moved off the telegram
_SIGNAL = re.compile(r"^(?P<lead>\s*SIGNAL\s+)(?P<name>\$[A-Z0-9_]+)(?P<sp>\s+)(?P<val>[^;\r\n]*?)"
                     r"(?P<tail>\s*(;[^\r\n]*)?)$", re.I)


def remap_machine_dat(text, mapping=None):
    """(new text, [(name, old, new)]): SIGNALs of `mapping` set to their input, every other SIGNAL on
    $IN[1..256] set to $IN[1026] (always FALSE), every SIGNAL on $OUT[1..256] moved to a spare output
    from $OUT[4081]. Idempotent; line endings and comments are kept."""
    mapping = {k.upper(): v for k, v in (mapping or MACHINE_DAT_MAP).items()}
    eol = eol_of(text)
    out, changes = [], []
    # System outputs on $OUT[1..256] (VW: $RC_RDY1 [9], $I_O_ACTCONF [12], $NEAR_POSRET [16],
    # $COLL_ALARM [40]) overlap bt2krl's signals there; KSS then refuses the writes (M_1388). Move
    # them to outputs the file does not use yet, from $OUT[4081].
    used = {int(n) for n in re.findall(r"\$OUT\[(\d+)\]", text, re.I)}
    spare = (n for n in range(OUT_SPARE, 4097) if n not in used)
    for line in text.split(eol):
        m = _SIGNAL.match(line)
        if not m:
            out.append(line)
            continue
        name, val = m.group("name").upper(), m.group("val").strip()
        want = mapping.get(name)
        if want is None:
            mi = re.fullmatch(r"\$IN\[(\d+)\]", val, re.I)
            if mi and int(mi.group(1)) in TELEGRAM_INPUTS:
                want = IN_FALSE
            mo = re.fullmatch(r"\$OUT\[(\d+)\]", val, re.I)
            if mo and int(mo.group(1)) in TELEGRAM_OUTPUTS:
                n = next(spare, None)
                if n is None:
                    raise SystemExit("no spare output left for %s" % name)
                want = "$OUT[%d]" % n
        if want is None or val.upper() == want.upper():
            out.append(line)
            continue
        tail = m.group("tail") or ""
        tail = re.sub(r"\s*;P2C was \S+", "", tail)
        note = ";P2C was %s" % val
        tail = (tail + " " + note) if tail.strip() else (" " + note)
        out.append("%s%s%s%s%s" % (m.group("lead"), m.group("name"), m.group("sp"), want, tail))
        changes.append((m.group("name"), val, want))
    return eol.join(out), changes


def machine_dat_path(instance):
    return os.path.join(INSTANCES, instance, "ROBOTER", "KRC", "STEU", "Mada", "$machine.dat")


def running_units(instance):
    """The KSS runtime units that run `instance` now (krc1: kss-runtime; krcN: kss-runtime-krcN)."""
    import glob
    import subprocess
    units = []
    if active_instance("krc1") == instance:
        units.append("kss-runtime.service")
    for f in sorted(glob.glob("/etc/kuka-ec2/krc*/active-instance.ini")):
        ns = os.path.basename(os.path.dirname(f))
        if ns != "krc1" and active_instance(ns) == instance:
            units.append("kss-runtime-%s.service" % ns)
    return [u for u in units
            if subprocess.run(["systemctl", "is-active", "--quiet", u]).returncode == 0]


def cmd_machine_dat(a):
    path = machine_dat_path(a.instance)
    if not a.dry_run and not a.force:
        # KSS keeps $machine.dat in memory and may write it back at shutdown: change it only while
        # the controller that runs this instance is stopped.
        running = running_units(a.instance)
        if running:
            emit({"ok": False, "error": "%s runs %s: stop it first (kuka-switch stop / kvc-add-controller.sh "
                  "stop) or give --force" % (", ".join(running), a.instance)}, 3)
    raw = open(path, "rb").read()
    text = raw.decode("latin-1")
    mapping = dict(MACHINE_DAT_MAP)
    if a.drives_off:
        mapping["$DRIVES_OFF"] = "$IN[1]"
    if a.move_enable:
        # $MOVE_ENABLE must not share the DrivesOn bit: the head drops DrivesOn during a reset for the
        # $DRIVES_ON edge, which would also drop the motion enable and raise M_203/M_1210 (acknowledge
        # needed again) -> no drives. $IN[1025] (always TRUE) is refused in EXT (M_249): use the head's
        # MoveEnable bit, $IN[8] (kvc-cell-setup MOVE_ENABLE_IN).
        mapping["$MOVE_ENABLE"] = a.move_enable
    new, changes = remap_machine_dat(text, mapping)
    res = {"ok": True, "file": path, "changes": [{"signal": n, "old": o, "new": w} for n, o, w in changes]}
    if not changes or a.dry_run:
        res["written"] = False
        emit(res)
    orig_dir = os.path.join(STATE_DIR, "kuka-vc", "machine-dat")
    os.makedirs(orig_dir, exist_ok=True)
    orig = os.path.join(orig_dir, a.instance + ".orig")
    if not os.path.exists(orig):
        with open(orig, "wb") as f:
            f.write(raw)
    ts = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    with open(os.path.join(orig_dir, "%s.%s" % (a.instance, ts)), "wb") as f:
        f.write(raw)
    st = os.stat(path)
    tmp = path + ".p2c-new"
    with open(tmp, "wb") as f:
        f.write(new.encode("latin-1"))
    os.chown(tmp, st.st_uid, st.st_gid)
    os.chmod(tmp, st.st_mode & 0o7777)
    os.replace(tmp, path)
    log_change("machine-dat %s: %s (original %s)" % (
        a.instance, ", ".join("%s %s->%s" % c for c in changes), orig))
    res["written"] = True
    res["original"] = orig
    emit(res)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest="cmd", required=True)
    for n in ("status", "deploy", "verify"):
        p = sp.add_parser(n)
        p.add_argument("--netns", default="krc1")
        p.add_argument("--instance", required=True)
        if n != "status":
            p.add_argument("--bundle", required=True)
        if n == "deploy":
            p.add_argument("--sps-base")
            p.add_argument("--force", action="store_true")
            p.add_argument("--dry-run", action="store_true")
    p = sp.add_parser("program")
    p.add_argument("action", choices=("show", "cancel", "select"))
    p.add_argument("--netns", default="krc1")
    p.add_argument("--name", default="BT_CELL", help="cancel: the only program it deselects")
    p.add_argument("--module", default="KRC:\\R1\\Program\\BT_CELL", help="select: the module")
    p.add_argument("--timeout", type=float, default=10)
    p = sp.add_parser("braketest-need")
    p.add_argument("--netns", default="krc1")
    p.add_argument("--status", help="the output of 'kuka-braketest status --json'")
    p = sp.add_parser("machine-dat")
    p.add_argument("--instance", required=True)
    p.add_argument("--drives-off", action="store_true")
    p.add_argument("--move-enable", metavar="$IN[n]", help="input for $MOVE_ENABLE (default $IN[1], DrivesOn)")
    p.add_argument("--force", action="store_true")
    p.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()
    if a.cmd != "machine-dat":
        enter_netns(a.netns)
    try:
        {"status": cmd_status, "deploy": cmd_deploy, "verify": cmd_verify, "program": cmd_program,
         "braketest-need": cmd_braketest_need, "machine-dat": cmd_machine_dat}[a.cmd](a)
    except SystemExit:
        raise
    except Exception as e:  # noqa: BLE001
        emit({"ok": False, "error": "%s: %s" % (type(e).__name__, e)}, 1)


if __name__ == "__main__":
    main()
