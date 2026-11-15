#!/usr/bin/env python3
"""p2c-headstub: plays plan2cell's head PLC for one KUKA VC controller in IO mode sim (test tool).

Talks to the controller's vass-io HTTP API (http://<kli>:8090, the $OUT[7000+n] -> $IN[n] loopback)
with the bt2plc simulator telegram (bt2krl profile sim, [sim] in_base = out_base = 0):

  PLAN_ID   $IN[33..48]  WORD LE, written once at start (--plan-id)
  GRANT b   $IN[97+b]    raised while REQUEST b is high, dropped after REQUEST b falls (auto-grant)
  REQUEST b $OUT[225+b]  read
  STEP      $OUT[33..48] read

The vass-io responder rules R1..R6 are switched off while the stub runs (R1 owns E33..E36, which
are PLAN_ID bits here) and restored on exit; the stub drops PLAN_ID and every GRANT on exit.
Every change of STEP / REQUEST / GRANT is printed as one JSON line (also to --log).

--t1-bco-unlatch (T1 tests only): in T1 the first motion of a selected Folge stops at
"Programmed path reached (BCO)" (M_1350, #P_STOP), and bt_sps latches BT_ABRT on any #P_STOP of a
running Folge. With this flag the stub clears that latch (SetVar BT_ABRT=FALSE through the kernel
API, so it must run in the controller netns: `ip netns exec krc1 ...`) once the interpreter is
#P_ACTIVE again, and logs it. Never used with EXT/AUT.

  ip netns exec krc1 python3 -I p2c-headstub.py --api http://10.200.0.11:8090 --plan-id 16#F77E \\
      [--duration 900] [--log /var/lib/plan2cell/headstub.log] [--t1-bco-unlatch]
"""
import argparse
import json
import os
import signal
import sys
import threading
import time
import urllib.request

KRL_DIR = "/opt/kuka-ec2/krl"
RULES = ("R1", "R2", "R3", "R4", "R5", "R6")


def bits(hexstr, first, n):
    """Integer of signals first..first+n-1 (signal k = bit (k-1)%8 of byte (k-1)//8)."""
    b = bytes.fromhex(hexstr)
    v = 0
    for i in range(n):
        k = first + i - 1
        if b[k // 8] >> (k % 8) & 1:
            v |= 1 << i
    return v


def plan_int(s):
    s = str(s).strip()
    return int(s[3:], 16) if s.startswith("16#") else int(s)


class Api:
    def __init__(self, base):
        self.base = base.rstrip("/")

    def get(self, path, timeout=15):
        with urllib.request.urlopen(self.base + path, timeout=timeout) as r:
            return json.loads(r.read())

    def post(self, path, body):
        req = urllib.request.Request(self.base + path, data=json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json"}, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=10) as r:
                return json.loads(r.read())
        except urllib.error.HTTPError as e:
            raise RuntimeError("%s %s: HTTP %d %s" % (path, body, e.code, e.read()[:200].decode(errors="replace")))

    def set_in(self, n, v):
        return self.post("/api/in", {"input": n, "value": 1 if v else 0})


class Stub:
    def __init__(self, a):
        self.a = a
        self.api = Api(a.api)
        self.t0 = time.time()
        self.logf = open(a.log, "a") if a.log else None
        self.stop = threading.Event()
        self.grant = 0
        self.saved_rules = {}
        self.events = []

    def out(self, **kw):
        kw = dict({"t": round(time.time() - self.t0, 3)}, **kw)
        line = json.dumps(kw, separators=(",", ":"))
        print(line, flush=True)
        if self.logf:
            self.logf.write(line + "\n")
            self.logf.flush()

    def start(self):
        rules = self.api.get("/api/rules")
        for r in RULES:
            if rules.get(r, {}).get("on"):
                self.saved_rules[r] = True
                self.api.post("/api/rules", {"rule": r, "on": False})
        plan = plan_int(self.a.plan_id)
        for i in range(16):
            self.api.set_in(33 + i, plan >> i & 1)
        for b in range(32):
            self.api.set_in(97 + b, 0)
        if self.a.drives:
            # DrivesOn level, as the head holds it: $machine.dat maps $DRIVES_ON and $MOVE_ENABLE to it.
            self.api.set_in(1, 1)
        self.out(ev="start", plan_id=plan, rules_off=sorted(self.saved_rules), drives=self.a.drives)

    def finish(self):
        # Best effort per step: the vass-io rules must come back even if an input write fails.
        errors = []
        for n in ([97 + b for b in range(32) if self.grant >> b & 1] + [33 + i for i in range(16)]
                  + ([1] if self.a.drives else [])):
            try:
                self.api.set_in(n, 0)
            except Exception as e:  # noqa: BLE001
                errors.append("in %d: %s" % (n, str(e)[:100]))
        for r in self.saved_rules:
            try:
                self.api.post("/api/rules", {"rule": r, "on": True})
            except Exception as e:  # noqa: BLE001
                errors.append("rule %s: %s" % (r, str(e)[:100]))
        self.out(ev="stop", rules_restored=sorted(self.saved_rules), errors=errors[:10])

    def run(self):
        seq, last = None, None
        end = self.t0 + self.a.duration
        while not self.stop.is_set() and time.time() < end:
            try:
                st = self.api.get("/api/state" + ("?since=%d" % seq if seq is not None else ""), timeout=15)
            except Exception as e:  # noqa: BLE001
                self.out(ev="error", error=str(e)[:200])
                time.sleep(1)
                continue
            seq = st["seq"]
            step = bits(st["out"], 33, 16)
            req = bits(st["out"], 225, 32)
            gin = bits(st["in"], 97, 32)
            planin = bits(st["in"], 33, 16)
            for b in range(32):
                m = 1 << b
                if req & m and not self.grant & m:
                    self.api.set_in(97 + b, 1)
                    self.grant |= m
                    self.out(ev="grant", bit=b)
                elif not req & m and self.grant & m:
                    self.api.set_in(97 + b, 0)
                    self.grant &= ~m
                    self.out(ev="release", bit=b)
            now = (step, req, gin, planin)
            if now != last:
                self.out(ev="io", step=step, request="16#%08X" % req, grant="16#%08X" % gin, plan_in="16#%04X" % planin)
                last = now

    def unlatch(self):
        sys.path.insert(0, KRL_DIR)
        from kssapi import KssApi
        api = KssApi(timeout=5)
        stopped = False
        while not self.stop.is_set():
            try:
                st, abrt, run, mode = (api.show_var(v) for v in ("$PRO_STATE1", "BT_ABRT", "BT_RUN", "$MODE_OP"))
            except Exception:  # noqa: BLE001
                time.sleep(0.5)
                continue
            if mode != "#T1":
                time.sleep(0.2)
                continue
            if st == "#P_STOP":
                stopped = True
            elif st == "#P_ACTIVE" and stopped and abrt == "TRUE" and run == "TRUE":
                api.set_var("BT_ABRT", "FALSE")
                self.out(ev="t1_bco_unlatch", note="BT_ABRT latched at a T1 #P_STOP cleared")
                stopped = False
            elif st == "#P_ACTIVE":
                stopped = False
            time.sleep(0.05)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--api", default="http://10.200.0.11:8090")
    ap.add_argument("--plan-id", required=True)
    ap.add_argument("--duration", type=float, default=900)
    ap.add_argument("--log")
    ap.add_argument("--t1-bco-unlatch", action="store_true")
    ap.add_argument("--drives", action="store_true",
                    help="hold $IN[1] (DrivesOn, also $MOVE_ENABLE after machine-dat) TRUE while running")
    a = ap.parse_args()
    s = Stub(a)
    signal.signal(signal.SIGTERM, lambda *_: s.stop.set())
    signal.signal(signal.SIGINT, lambda *_: s.stop.set())
    try:
        s.start()   # inside the try: rules switched off before a failure are restored by finish()
        if a.t1_bco_unlatch:
            threading.Thread(target=s.unlatch, daemon=True).start()
        s.run()
    finally:
        s.stop.set()
        s.finish()


if __name__ == "__main__":
    main()
