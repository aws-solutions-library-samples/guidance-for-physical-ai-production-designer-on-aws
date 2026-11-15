#!/usr/bin/env python3
"""The post-install smoke test of a FactoryOS public deployment (deployment/README.md, plan Phase 3).

Run it from a developer machine, with credentials for the account the stack is in, after `install.sh status` says
ready:

    python3 -m venv /tmp/smoke-venv && /tmp/smoke-venv/bin/pip install -r deployment/test/requirements-smoke.txt
    /tmp/smoke-venv/bin/python -m playwright install chromium
    /tmp/smoke-venv/bin/python -I deployment/test/smoke.py --stack-name factoryos --region eu-central-1

What it checks, in this order (any failure exits 1; the cleanup runs whatever happens):
  1. the denial matrix, as an anonymous client (the VerifyAnonymousDenial custom resource's checks,
     lambda/verify-denial/index.mjs): a page load of / goes to the pool's /oauth2/authorize with this client, PKCE and
     state; /api/*, HEAD, POST and WebSocket upgrades get 401 with the gate's WWW-Authenticate; a foreign Origin's
     upgrade is refused; garbage, alg=none, expired and wrong-pool tokens are denied; the callback without its state is
     refused; nothing answers 2xx or 101
  2. a temporary user smoke-<uuid>@example.com (RFC 2606 reserved: no mail can go anywhere; MessageAction=SUPPRESS
     anyway), a permanent generated password held in memory only, TOTP enrolled through the user pool's own SRP sign-in
     and pyotp; first, every smoke-* user older than 1 h (a run that could not clean up) is deleted
  3. a sign-in through the managed login in a fresh headless Chromium: no tracing, no HAR, no stored state
  4. signed in: 200 on / and /api/v1/health; the /zenoh and /chat WebSocket upgrades open (8 tries, 10 s apart);
     an upload (the user's storage scope) and a catalogue write (the catalogue scope; the default user is a catalogue admin), each read back
     and deleted; the agentic status through the edge; on the instance (SSM Run Command): one @factoryos MCP tool
     (factoryos-mcp over stdio, as the factoryos user, read-only agentic_status) and one planner router call (its
     /health and its MCP tools/list on 127.0.0.1:8893); then umarell_jam, when the release ships it (its router script
     and the cell planner's umarell_jam/) and the host does not turn it off: as the factoryos user, the plain umarell
     76ea13a the router's gate looks for (umarell_jam.jam_source) is there, its venv imports the numerical runtime the
     gate checks (no "numerical runtime is incomplete"), `python -m umarell_jam --root <new temp dir> --umarell <that
     tree> setup` exits 0 within 180 s, and describe_planner umarell_jam (--jam-cell <cell>: for that cell, which must
     have a task program) shows neither of the gate's setup warnings. A release that ships umarell_jam without it set
     up fails, whatever provisioning marked incomplete
  5. the listener allowlist on the instance (SSM: ss -Hltnu): only loopback, plus TCP :80 (the backend, which only the
     CloudFront VPC origin reaches) and the DHCP clients' UDP :68/:546 bound to an interface; again after Start
     Simulation (the umarell planner MCP must stay on loopback), unless --skip-sim
  6. sign-out (/_auth/logout; the API is 401 again), then the user deleted: in a finally, at exit and on SIGINT /
     SIGTERM / SIGHUP; a simulation it started is stopped
Nothing secret is printed or passed in an argv: the password, the TOTP secret and the tokens stay in this process.

--self-test runs the offline parts against a stub gate on 127.0.0.1 (no AWS, no browser): the denial matrix on a
correct, a leaky and a misconfigured gate, the SRP arithmetic against a server-side computation, the listener
allowlist, the password policy, the sweep's selection and (with pyotp installed) TOTP.

Needs: python3 >= 3.9; boto3, pyotp and playwright (requirements-smoke.txt) and Chromium (playwright install) for a
real run. The AWS calls: cloudformation:DescribeStacks; cognito-idp:AdminCreateUser, AdminSetUserPassword,
AdminSetUserMFAPreference, AdminDeleteUser, ListUsers on the pool (InitiateAuth, RespondToAuthChallenge,
AssociateSoftwareToken, VerifySoftwareToken, RevokeToken are the public client's); ssm:SendCommand
(AWS-RunShellScript) and ssm:GetCommandInvocation on the instance.
"""

from __future__ import annotations

import argparse
import atexit
import base64
import datetime as dt
import hashlib
import hmac
import http.client
import ipaddress
import json
import os
import re
import secrets
import signal
import socket
import string
import sys
import threading
import time
import urllib.parse
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# --- constants shared with the gate (lambda/edge-auth/index.mjs) -------------------------------------------------
ID_COOKIE = "__Host-fos_id"
ACCESS_COOKIE = "__Host-fos_access"
REALM = 'realm="factoryos"'
LOGOUT_PATH = "/_auth/logout"
CALLBACK_PATH = "/_auth/callback"
HEALTH = "/api/v1/health"
WS_TRIES = 8
WS_RETRY_S = 10
SMOKE_DOMAIN = "example.com"                    # RFC 2606: reserved, no mailbox
SMOKE_EMAIL = re.compile(r"smoke-[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}@example\.com")
SWEEP_AGE = dt.timedelta(hours=1)
MCP_BIN = "/opt/factoryos/current/bin/factoryos-mcp"
ROUTER = "http://127.0.0.1:8893"
JAM_SETUP_S = 180                               # umarell_jam's setup builds R/umarell (~60 s cold on m7g.2xlarge)
# The router's gate warnings for umarell_jam (deploy/planner-router/server.py _gate_jam) that mean it is not set up.
JAM_WARNINGS = ("numerical runtime is incomplete", "no plain umarell")


class SmokeError(Exception):
    """A check that failed (the message says which and why; never a secret)."""


def say(msg: str) -> None:
    print(msg, flush=True)


def ok(msg: str) -> None:
    say(f"ok    {msg}")


# --- 1. the denial matrix (standard library only) ------------------------------------------------------------------
def b64url(data: bytes) -> str:
    return base64.urlsafe_b64encode(data).rstrip(b"=").decode()


def b64json(obj) -> str:
    return b64url(json.dumps(obj, separators=(",", ":")).encode())


def forged_tokens(issuer: str, client_id: str, region: str, now: float | None = None) -> dict:
    """Tokens no gate may accept: garbage, unsigned (alg=none), expired, of another pool. The "signatures" are random
    bytes of an RS256 signature's length: no key the gate trusts made them (its kid is not in the pool's JWKS), which is
    all a real signature by an unrelated key would show the gate too."""
    t = int(now if now is not None else time.time())

    def claims(use: str, **extra) -> dict:
        c = {"sub": "00000000-0000-4000-8000-000000000000", "iss": issuer, "token_use": use,
             **({"aud": client_id} if use == "id" else {"client_id": client_id}), "iat": t - 60, "exp": t + 3600}
        c.update(extra)
        return c

    def signed(c: dict) -> str:
        return f"{b64json({'alg': 'RS256', 'kid': 'smoke-forged'})}.{b64json(c)}.{b64url(secrets.token_bytes(256))}"

    other = f"https://cognito-idp.{region}.amazonaws.com/{region}_smokeOtherPool"
    return {
        "garbage": {"id": "not-a-token", "access": f"{b64json({'alg': 'RS256'})}.{b64json({})}.AAAA"},
        "alg-none": {u: f"{b64json({'alg': 'none', 'typ': 'JWT'})}.{b64json(claims(u))}." for u in ("id", "access")},
        "expired": {u: signed(claims(u, iat=t - 7200, exp=t - 3600)) for u in ("id", "access")},
        "wrong-pool": {u: signed(claims(u, iss=other)) for u in ("id", "access")},
    }


def http_request(base: str, method: str, path: str, headers: dict, timeout: float = 15.0) -> tuple[int, dict]:
    """One request, no redirects followed: (status, lower-cased headers); 101 for an upgraded WebSocket."""
    u = urllib.parse.urlsplit(base)
    cls = http.client.HTTPSConnection if u.scheme == "https" else http.client.HTTPConnection
    conn = cls(u.hostname, u.port, timeout=timeout)
    try:
        conn.putrequest(method, path, skip_accept_encoding=True)
        for k, v in headers.items():
            conn.putheader(k, v)
        conn.endheaders()
        r = conn.getresponse()
        h = {}
        for k, v in r.getheaders():
            h[k.lower()] = f"{h[k.lower()]}, {v}" if k.lower() in h else v
        return r.status, h
    finally:
        conn.close()


def matrix(base: str, client_id: str, cognito_domain: str, issuer: str, region: str, now: float | None = None) -> list:
    """The checks: [(name, method, path, headers, expect)], expect(status, headers) -> problem or None."""
    page = {"accept": "text/html,application/xhtml+xml", "sec-fetch-mode": "navigate", "sec-fetch-dest": "document"}

    def ws(origin: str) -> dict:
        return {"connection": "Upgrade", "upgrade": "websocket", "sec-websocket-version": "13",
                "sec-websocket-key": base64.b64encode(secrets.token_bytes(16)).decode(), "origin": origin}

    def to_login(status: int, h: dict):
        if status != 302:
            return f"expected 302 to the login, got {status}"
        u = urllib.parse.urlsplit(h.get("location", ""))
        q = urllib.parse.parse_qs(u.query)
        if f"{u.scheme}://{u.netloc}" != f"https://{cognito_domain}" or u.path != "/oauth2/authorize":
            return f"redirect to {u.scheme}://{u.netloc}{u.path}"
        if q.get("client_id") != [client_id]:
            return "redirect with another client_id"
        if q.get("redirect_uri") != [f"{base}{CALLBACK_PATH}"]:
            return "redirect with another redirect_uri"
        if q.get("code_challenge_method") != ["S256"] or not q.get("state"):
            return "redirect without PKCE/state"
        return None

    def gate401(status: int, h: dict):
        if status != 401:
            return f"expected 401, got {status}"
        return None if REALM in h.get("www-authenticate", "") else "a 401 without the gate's WWW-Authenticate"

    def refused(status: int, h: dict):
        return None if status in (401, 403) else f"expected 401/403, got {status}"

    def client_error(status: int, h: dict):
        return None if 400 <= status < 500 else f"expected 4xx, got {status}"

    checks = [
        ("page load of /", "GET", "/", page, to_login),
        ("API GET", "GET", HEALTH, {"accept": "application/json"}, gate401),
        ("HEAD /", "HEAD", "/", {}, gate401),
        ("API POST", "POST", HEALTH, {"origin": base, "content-length": "0"}, gate401),
        ("WebSocket /zenoh", "GET", "/zenoh", ws(base), gate401),
        ("WebSocket /chat", "GET", "/chat", ws(base), gate401),
        ("WebSocket, foreign Origin", "GET", "/chat", ws("https://attacker.example"), refused),
        ("callback without state", "GET", f"{CALLBACK_PATH}?code=x&state=y", page, client_error),
    ]
    for kind, t in forged_tokens(issuer, client_id, region, now).items():
        cookie = f"{ID_COOKIE}={t['id']}; {ACCESS_COOKIE}={t['access']}"
        checks.append((f"{kind} token, API", "GET", HEALTH, {"cookie": cookie}, gate401))
        checks.append((f"{kind} token, page load", "GET", "/", {**page, "cookie": cookie}, to_login))
    return checks


def run_matrix(base: str, props: dict, request=http_request) -> tuple[str | None, list[str], int]:
    """One pass: (fatal, problems, checks run). A 2xx or 101 anywhere is fatal at once."""
    problems, n = [], 0
    for name, method, path, headers, expect in matrix(base, **props):
        n += 1
        try:
            status, h = request(base, method, path, headers)
        except (OSError, http.client.HTTPException) as e:
            problems.append(f"{name}: {type(e).__name__}: {e}")
            continue
        if 200 <= status < 300 or status == 101:
            return f"{name}: answered {status} without a session", problems, n
        p = expect(status, h)
        if p:
            problems.append(f"{name}: {p}")
    return None, problems, n


def check_denial(base: str, props: dict, rounds: int = 3, wait_s: float = 10.0, request=http_request) -> int:
    """The matrix until it passes (a few rounds: a transient error is not a gate's answer); fatal at once."""
    for r in range(1, rounds + 1):
        fatal, problems, n = run_matrix(base, props, request)
        if fatal:
            raise SmokeError(f"the gate let an anonymous request through: {fatal}")
        if not problems:
            return n
        say(f"..    denial matrix round {r}: {'; '.join(problems[:4])}")
        if r < rounds:
            time.sleep(wait_s)
    raise SmokeError(f"the denial matrix did not pass: {'; '.join(problems[:6])}")


# --- 2. the temporary user: SRP sign-in (USER_SRP_AUTH, the app client's only flow) and TOTP --------------------------
# RFC 5054's 3072-bit group, as Cognito uses it (its public prime: the pragmas tell detect-secrets it is no secret).
N_HEX = (
    "FFFFFFFFFFFFFFFFC90FDAA22168C234C4C6628B80DC1CD129024E088A67CC74020BBEA63B139B22514A08798E3404DDEF9519B3CD3A431B"  # pragma: allowlist secret
    "302B0A6DF25F14374FE1356D6D51C245E485B576625E7EC6F44C42E9A637ED6B0BFF5CB6F406B7EDEE386BFB5A899FA5AE9F24117C4B1FE6"  # pragma: allowlist secret
    "49286651ECE45B3DC2007CB8A163BF0598DA48361C55D39A69163FA8FD24CF5F83655D23DCA3AD961C62F356208552BB9ED529077096966D"  # pragma: allowlist secret
    "670C354E4ABC9804F1746C08CA18217C32905E462E36CE3BE39E772C180E86039B2783A2EC07A28FB5C55DF06F4C52C9DE2BCBF695581718"  # pragma: allowlist secret
    "3995497CEA956AE515D2261898FA051015728E5A8AAAC42DAD33170D04507A33A85521ABDF1CBA64ECFB850458DBEF0A8AEA71575D060C7D"  # pragma: allowlist secret
    "B3970F85A6E1E4C7ABF5AE8CDB0933D71E8C94E04A25619DCEE3D2261AD2EE6BF12FFA06D98A0864D87602733EC86A64521F2B18177B200C"  # pragma: allowlist secret
    "BBE117577A615D6C770988C0BAD946E208E24FA074E5AB3143DB5BFCE0FD108E4B82D120A93AD2CAFFFFFFFFFFFFFFFF"  # pragma: allowlist secret
)
N = int(N_HEX, 16)
G = 2
INFO = b"Caldera Derived Key"


def _hash_hex(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest().rjust(64, "0")


def pad_hex(v) -> str:
    """The hex of an int (or a hex string) as Cognito hashes it: even length, a leading 00 when the top bit is set."""
    h = format(v, "x") if isinstance(v, int) else v
    if len(h) % 2:
        return "0" + h
    return "00" + h if h[0] in "89abcdefABCDEF" else h


def _hex_hash(hex_str: str) -> str:
    return _hash_hex(bytes.fromhex(hex_str))


K = int(_hex_hash("00" + N_HEX + "0" + format(G, "x")), 16)


def srp_a(a: int) -> int:
    return pow(G, a, N)


def srp_x(pool_name: str, user_id: str, password: str, salt_hex: str) -> int:
    return int(_hex_hash(pad_hex(salt_hex) + _hash_hex(f"{pool_name}{user_id}:{password}".encode())), 16)


def srp_u(big_a: int, big_b: int) -> int:
    return int(_hex_hash(pad_hex(big_a) + pad_hex(big_b)), 16)


def srp_client_s(a: int, big_b: int, x: int, u: int) -> int:
    if big_b % N == 0 or u == 0:
        raise SmokeError("SRP: the user pool's B or u is 0")
    return pow(big_b - K * pow(G, x, N), a + u * x, N)


def hkdf16(ikm: bytes, salt: bytes) -> bytes:
    prk = hmac.new(salt, ikm, hashlib.sha256).digest()
    return hmac.new(prk, INFO + b"\x01", hashlib.sha256).digest()[:16]


def srp_timestamp(now: dt.datetime | None = None) -> str:
    """'Tue Nov 3 09:05:01 UTC 2026': the day without a leading zero."""
    now = now or dt.datetime.now(dt.timezone.utc)
    day = ("Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun")[now.weekday()]
    month = ("Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec")[now.month - 1]
    return f"{day} {month} {now.day} {now:%H:%M:%S} UTC {now.year}"   # English names whatever the locale


def srp_claim(pool_id: str, a: int, big_a: int, params: dict, password: str, now: dt.datetime | None = None) -> dict:
    """The PASSWORD_VERIFIER challenge's responses for the challenge parameters `params`."""
    pool_name = pool_id.split("_", 1)[1]
    user_id = params["USER_ID_FOR_SRP"]
    big_b = int(params["SRP_B"], 16)
    u = srp_u(big_a, big_b)
    s = srp_client_s(a, big_b, srp_x(pool_name, user_id, password, params["SALT"]), u)
    key = hkdf16(bytes.fromhex(pad_hex(s)), bytes.fromhex(pad_hex(u)))
    stamp = srp_timestamp(now)
    msg = pool_name.encode() + user_id.encode() + base64.b64decode(params["SECRET_BLOCK"]) + stamp.encode()
    return {"USERNAME": params.get("USERNAME", user_id), "TIMESTAMP": stamp, "PASSWORD_CLAIM_SECRET_BLOCK": params["SECRET_BLOCK"],
            "PASSWORD_CLAIM_SIGNATURE": base64.b64encode(hmac.new(key, msg, hashlib.sha256).digest()).decode()}


def make_password(n: int = 32) -> str:
    """A password the pool's policy takes (14+, lower, upper, digit, symbol): `secrets`, in memory only."""
    symbols = "^$*.[]{}()?-!@#%&/,><:;|_~+="
    alphabet = string.ascii_letters + string.digits + symbols
    while True:
        p = "".join(secrets.choice(alphabet) for _ in range(n))
        if (any(c.islower() for c in p) and any(c.isupper() for c in p) and any(c.isdigit() for c in p)
                and any(c in symbols for c in p)):
            return p


def sweep_candidates(users: list, now: dt.datetime) -> list[str]:
    """The user names of smoke users older than SWEEP_AGE (only exact smoke-<uuid>@example.com e-mails)."""
    out = []
    for user in users:
        attrs = {a["Name"]: a["Value"] for a in user.get("Attributes", [])}
        created = user.get("UserCreateDate")
        if SMOKE_EMAIL.fullmatch(attrs.get("email", "")) and created and now - created > SWEEP_AGE:
            out.append(user["Username"])
    return out


# --- 5. the listener allowlist -----------------------------------------------------------------------------------
DHCP_CLIENT_PORTS = {68, 546}


def _host_port(local: str) -> tuple[str, int, str]:
    host, _, port = local.rpartition(":")
    host, _, iface = host.partition("%")
    return host.strip("[]"), int(port) if port.isdigit() else -1, iface


def _loopback(host: str) -> bool:
    try:
        ip = ipaddress.ip_address(host)
    except ValueError:
        return host == "localhost"
    mapped = getattr(ip, "ipv4_mapped", None)
    return ip.is_loopback or bool(mapped and mapped.is_loopback)


def listener_problems(ss_output: str) -> tuple[list[str], int]:
    """`ss -Hltnu` lines that are not allowed: anything but loopback, TCP :80 on any address, and the DHCP clients'
    UDP :68/:546 bound to an interface (outbound clients; the security group admits only :80 anyway)."""
    bad, n = [], 0
    for line in ss_output.splitlines():
        f = line.split()
        if len(f) < 5:
            continue
        n += 1
        proto, local = f[0], f[4]
        host, port, iface = _host_port(local)
        if _loopback(host):
            continue
        if proto == "tcp" and port == 80:
            continue
        if proto == "udp" and port in DHCP_CLIENT_PORTS and iface:
            continue
        bad.append(f"{proto} {local}" + (f" ({' '.join(f[6:])})" if len(f) > 6 else ""))
    return bad, n


# --- the instance over SSM Run Command ---------------------------------------------------------------------------
def ssm_run(ssm, instance_id: str, script: str, timeout_s: int = 300) -> str:
    """AWS-RunShellScript on the instance; its stdout (fails on a non-zero exit)."""
    cid = ssm.send_command(InstanceIds=[instance_id], DocumentName="AWS-RunShellScript", Comment="factoryos smoke.py",
                           TimeoutSeconds=max(30, timeout_s), Parameters={"commands": [script]})["Command"]["CommandId"]
    deadline = time.time() + timeout_s + 60
    while True:
        time.sleep(3)
        try:
            inv = ssm.get_command_invocation(CommandId=cid, InstanceId=instance_id)
        except ssm.exceptions.InvocationDoesNotExist:
            inv = {"Status": "Pending"}
        if inv["Status"] in ("Success", "Failed", "Cancelled", "TimedOut", "Cancelling"):
            break
        if time.time() > deadline:
            raise SmokeError(f"SSM command {cid} did not finish in {timeout_s} s")
    if inv["Status"] != "Success":
        raise SmokeError(f"SSM command {cid}: {inv['Status']}: {(inv.get('StandardErrorContent') or '')[-800:]}")
    return inv.get("StandardOutputContent") or ""


MCP_SCRIPT = r"""set -eu
req() { printf '%s\n' "$@"; }
{ req '{"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2024-11-05","capabilities":{},"clientInfo":{"name":"smoke","version":"1"}}}' \
      '{"jsonrpc":"2.0","method":"notifications/initialized"}' \
      '{"jsonrpc":"2.0","id":2,"method":"tools/call","params":{"name":"agentic_status","arguments":{}}}'
  sleep 20; } | timeout 60 runuser -u factoryos -- __MCP__ 2>/dev/null | grep -m1 '"id":2' || { echo 'no MCP answer' >&2; exit 1; }
"""

ROUTER_SCRIPT = r"""set -eu
curl -fsS --max-time 20 __ROUTER__/health >/dev/null
curl -fsS --max-time 30 -H 'Content-Type: application/json' -H 'Accept: application/json' \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/list","params":{}}' __ROUTER__/mcp
"""


def check_mcp(ssm, iid: str) -> None:
    out = ssm_run(ssm, iid, MCP_SCRIPT.replace("__MCP__", MCP_BIN), 120)
    try:
        r = json.loads(out.strip().splitlines()[-1])
    except (ValueError, IndexError):
        raise SmokeError(f"factoryos-mcp: no JSON-RPC answer ({out[-300:]!r})") from None
    res = r.get("result") or {}
    if "error" in r or res.get("isError"):
        raise SmokeError(f"factoryos-mcp agentic_status failed: {json.dumps(r)[:400]}")
    ok("@factoryos MCP: agentic_status answered (factoryos-mcp over stdio, as factoryos)")


def check_router(ssm, iid: str) -> None:
    out = ssm_run(ssm, iid, ROUTER_SCRIPT.replace("__ROUTER__", ROUTER), 90)
    try:
        tools = json.loads(out)["result"]["tools"]
    except (ValueError, KeyError, TypeError):
        raise SmokeError(f"planner router: no tools/list answer ({out[-300:]!r})") from None
    if not tools:
        raise SmokeError("planner router: tools/list is empty")
    ok(f"planner router: /health 200, MCP tools/list {len(tools)} tools")


# umarell_jam on the instance: one python program, run as root by SSM; it runs every step as the factoryos user and
# prints one JSON report (the verdict is jam_verdict's, testable offline). Its root is a new temporary directory,
# deleted again: the router's own <work>/umarell_jam is never touched.
JAM_SCRIPT = r"""set -eu
python3 -I - <<'FOS_JAM_PY'
import json, os, pwd, shutil, subprocess, tempfile
cur, root = "/opt/factoryos/current", "/opt/factoryos-cell-planner/router"
router_dir, src, py = cur + "/deploy/planner-router", root + "/src", root + "/jam-venv/bin/python"
umarell, setup_s, cell = "/var/lib/factoryos/umarell", __SETUP_S__, __CELL__
u = pwd.getpwnam("factoryos")
env = {"PATH": "/usr/local/bin:/usr/bin:/bin", "HOME": u.pw_dir, "LANG": "C.UTF-8", "PYTHONDONTWRITEBYTECODE": "1",
       "OMP_NUM_THREADS": "1"}
def as_factoryos(cmd, cwd, timeout):
    # timeout(1) under runuser: a command past its time is killed with its children (exit 124), not left running.
    try:
        r = subprocess.run(["runuser", "-u", "factoryos", "--", "env", "-i"] + ["%s=%s" % kv for kv in env.items()]
                           + ["timeout", "-k", "10", str(timeout)] + cmd,
                           cwd=cwd, capture_output=True, text=True, timeout=timeout + 30, stdin=subprocess.DEVNULL)
        out = (r.stdout + r.stderr).strip()
        return r.returncode, (out + "\n" if out else "") + ("timed out after %d s" % timeout if r.returncode == 124 else "")
    except subprocess.TimeoutExpired:
        return 124, "timed out after %d s" % timeout
rep = {"shipped": os.path.isfile(router_dir + "/umarell_jam.py") and os.path.isfile(cur + "/cell-planner/umarell_jam/__main__.py")}
flags = subprocess.run(["bash", "-c", '. /etc/factoryos/workspace.env 2>/dev/null; echo "${FACTORY_OS_CELL_PLANNER:-1} ${FACTORY_OS_UMARELL_JAM:-1}"'],
                       capture_output=True, text=True).stdout.split()
rep["off"] = "0" in flags
try:
    rep["incomplete"] = "umarell_jam" in open("/etc/factoryos/provision-incomplete").read().split()
except OSError:
    rep["incomplete"] = False
if rep["shipped"] and not rep["off"]:
    rc, out = as_factoryos(["/usr/bin/python3", "-I", "-c", "import sys; sys.path.insert(0, sys.argv[1]); "
                            "from pathlib import Path; import umarell_jam; s = umarell_jam.jam_source(Path(sys.argv[2]).resolve()); "
                            "print(s or '')", router_dir, umarell], router_dir, 60)
    rep["source"] = out.splitlines()[-1].strip() if rc == 0 and out else ""
    rep["source_error"] = "" if rc == 0 else out[-300:]
    rc, out = as_factoryos([py, "-W", "ignore", "-c", "import numpy, pinocchio, hppfcl, scipy, ompl, toppra"], src, 120)
    rep["runtime"] = "" if rc == 0 else (out.splitlines() or ["exit %d" % rc])[-1][:300]
    if rep["source"] and not rep["runtime"]:
        tmp = tempfile.mkdtemp(prefix="fos-smoke-jam-")
        try:
            os.chown(tmp, u.pw_uid, u.pw_gid)
            rc, out = as_factoryos([py, "-W", "ignore", "-m", "umarell_jam", "--root", tmp, "--umarell", rep["source"],
                                    "setup"], src, setup_s)
            rep["setup_rc"], rep["setup_tail"] = rc, out[-400:]
        finally:
            shutil.rmtree(tmp, ignore_errors=True)
    args = {"name": "umarell_jam"}
    if cell:
        args["cell"] = cell
    rc, out = as_factoryos(["curl", "-fsS", "--max-time", "60", "-H", "Content-Type: application/json", "-H",
                            "Accept: application/json", "-d", json.dumps({"jsonrpc": "2.0", "id": 1, "method": "tools/call",
                            "params": {"name": "describe_planner", "arguments": args}}), "http://127.0.0.1:8893/mcp"], "/", 90)
    rep["describe"] = out if rc == 0 else ""
    rep["describe_error"] = "" if rc == 0 else out[-300:]
print(json.dumps(rep))
FOS_JAM_PY
"""


def jam_script(cell: str | None) -> str:
    """JAM_SCRIPT with its placeholders filled (python literals: the cell is a repr, never shell-quoted text)."""
    if cell is not None and not re.fullmatch(r"[A-Za-z0-9._-]{1,128}", cell):
        raise SmokeError(f"--jam-cell {cell!r} is not a cell name")
    return JAM_SCRIPT.replace("__SETUP_S__", str(JAM_SETUP_S)).replace("__CELL__", repr(cell))


def jam_verdict(rep: dict, cell: str | None = None) -> tuple[str, str]:
    """('skip' | 'ok' | 'fail', why) of JAM_SCRIPT's report: skip when the release does not ship umarell_jam or the
    host turns it off; fail when any of the gate's setup conditions, the setup run or describe_planner says it is not
    set up (provisioning's own provision-incomplete mark is named, never an excuse)."""
    if not rep.get("shipped"):
        return "skip", "this release does not ship umarell_jam"
    if rep.get("off"):
        return "skip", "umarell_jam (or the cell planner) is off on this host (workspace.env)"
    marked = " (provisioning marked it incomplete: /etc/factoryos/provision-incomplete)" if rep.get("incomplete") else ""
    if not rep.get("source"):
        why = rep.get("source_error") or "umarell_jam.jam_source found none next to /var/lib/factoryos/umarell"
        return "fail", f"no plain umarell 76ea13a for umarell_jam: {why}{marked}"
    if rep.get("runtime"):
        return "fail", f"umarell_jam's numerical runtime is incomplete in its venv: {rep['runtime']}{marked}"
    if rep.get("setup_rc") != 0:
        tail = ((rep.get("setup_tail") or "").strip().splitlines() or ["no output"])[-1]
        return "fail", f"umarell_jam setup (--umarell {rep['source']}) exited {rep.get('setup_rc')}: {tail}{marked}"
    try:
        r = json.loads(rep.get("describe") or "")
        res = r["result"]
        entry = json.loads(res["content"][0]["text"])
    except (ValueError, KeyError, IndexError, TypeError):
        return "fail", f"describe_planner umarell_jam: no answer ({(rep.get('describe_error') or rep.get('describe') or '')[-200:]!r})"
    if res.get("isError") or entry.get("name") != "umarell_jam":
        return "fail", f"describe_planner umarell_jam: {json.dumps(entry)[:300]}"
    if cell and not entry.get("available"):
        return "fail", f"umarell_jam is not available for cell {cell}: {entry.get('why_unavailable')}"
    bad = [w for w in entry.get("warnings") or [] if any(k in w for k in JAM_WARNINGS)]
    if bad:
        return "fail", f"describe_planner umarell_jam warns: {bad[0][:300]}"
    return "ok", (f"umarell_jam: setup on {rep['source']} exited 0, describe_planner"
                  f"{' for ' + cell if cell else ''} without setup warnings")


def check_jam(ssm, iid: str, cell: str | None = None) -> None:
    script = jam_script(cell)
    out = ssm_run(ssm, iid, script, JAM_SETUP_S + 240)
    try:
        rep = json.loads(out.strip().splitlines()[-1])
    except (ValueError, IndexError):
        raise SmokeError(f"umarell_jam: no report from the instance ({out[-300:]!r})") from None
    verdict, why = jam_verdict(rep, cell)
    if verdict == "fail":
        raise SmokeError(why)
    if verdict == "skip":
        say(f"skip  umarell_jam: {why}")
    else:
        ok(why)


def check_listeners(ssm, iid: str, when: str) -> None:
    out = ssm_run(ssm, iid, "ss -Hltnu", 60)
    bad, n = listener_problems(out)
    if bad:
        raise SmokeError(f"listeners {when}: not loopback or :80: {'; '.join(bad)}")
    ok(f"listeners {when}: {n} sockets, only loopback and :80")


# --- the cleanup: once, in a finally, at exit and on signals --------------------------------------------------------
class Cleanup:
    def __init__(self) -> None:
        self._steps: list = []
        self._lock = threading.Lock()
        atexit.register(self.run)
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, self._on_signal)

    def add(self, name: str, fn, first: bool = False) -> None:
        """`fn` at cleanup: the `first` ones before the rest (the user's deletion: nothing may hang before it), each
        group last-in first-out."""
        self._steps.append((not first, len(self._steps), name, fn))

    def _on_signal(self, signum, _frame) -> None:
        say(f"..    signal {signum}: cleaning up")
        self.run()
        os._exit(128 + signum)

    def run(self) -> None:
        with self._lock:
            steps, self._steps = self._steps, []
        for _, _, name, fn in sorted(steps, key=lambda s: (s[0], -s[1])):
            try:
                fn()
                ok(f"cleanup: {name}")
            except Exception as e:  # noqa: BLE001 - every step is tried; the error is shown
                say(f"FAIL  cleanup: {name}: {type(e).__name__}: {e}")


# --- the live run --------------------------------------------------------------------------------------------------
def stack_outputs(cfn, stack: str) -> dict:
    s = cfn.describe_stacks(StackName=stack)["Stacks"][0]
    if not s["StackStatus"].endswith("_COMPLETE") or "ROLLBACK" in s["StackStatus"]:
        raise SmokeError(f"stack {stack} is {s['StackStatus']}")
    out = {o["OutputKey"]: o["OutputValue"] for o in s.get("Outputs", [])}
    missing = [k for k in ("Url", "UserPoolId", "UserPoolClientId", "UserPoolDomain", "InstanceId") if not out.get(k)]
    if missing:
        raise SmokeError(f"stack {stack} has no output {', '.join(missing)}")
    if out.get("GateVerified") != "true":
        raise SmokeError(f"stack {stack}: GateVerified is {out.get('GateVerified')!r}, not true")
    return out


def sweep(idp, pool: str) -> int:
    now = dt.datetime.now(dt.timezone.utc)
    users, token = [], None
    while True:
        kw = {"UserPoolId": pool, "Filter": 'email ^= "smoke-"', "Limit": 60}
        if token:
            kw["PaginationToken"] = token
        r = idp.list_users(**kw)
        users += r.get("Users", [])
        token = r.get("PaginationToken")
        if not token:
            break
    old = sweep_candidates(users, now)
    for name in old:
        idp.admin_delete_user(UserPoolId=pool, Username=name)
    return len(old)


def create_user(idp, pool: str, client_id: str, email: str, password: str, pyotp) -> str:
    """The user, its permanent password and TOTP (via SRP sign-in and MFA_SETUP); returns the TOTP secret."""
    idp.admin_create_user(UserPoolId=pool, Username=email, MessageAction="SUPPRESS",
                          UserAttributes=[{"Name": "email", "Value": email}, {"Name": "email_verified", "Value": "true"}],
                          TemporaryPassword=make_password())
    idp.admin_set_user_password(UserPoolId=pool, Username=email, Password=password, Permanent=True)
    a = secrets.randbelow(N - 2) + 1
    big_a = srp_a(a)
    r = idp.initiate_auth(ClientId=client_id, AuthFlow="USER_SRP_AUTH",
                          AuthParameters={"USERNAME": email, "SRP_A": format(big_a, "x")})
    if r.get("ChallengeName") != "PASSWORD_VERIFIER":
        raise SmokeError(f"SRP sign-in: challenge {r.get('ChallengeName')!r}, not PASSWORD_VERIFIER")
    # Cognito answers USER_SRP_AUTH's PASSWORD_VERIFIER without a Session; an empty one fails botocore's min length 20.
    extra = {"Session": r["Session"]} if r.get("Session") else {}
    r = idp.respond_to_auth_challenge(ClientId=client_id, ChallengeName="PASSWORD_VERIFIER", **extra,
                                      ChallengeResponses=srp_claim(pool, a, big_a, r["ChallengeParameters"], password))
    if r.get("ChallengeName") != "MFA_SETUP":
        raise SmokeError(f"SRP sign-in: challenge {r.get('ChallengeName')!r}, not MFA_SETUP (is TOTP MFA required?)")
    assoc = idp.associate_software_token(Session=r["Session"])
    secret = assoc["SecretCode"]
    v = idp.verify_software_token(Session=assoc["Session"], UserCode=pyotp.TOTP(secret).now(), FriendlyDeviceName="smoke")
    if v.get("Status") != "SUCCESS":
        raise SmokeError(f"TOTP enrolment: {v.get('Status')}")
    r = idp.respond_to_auth_challenge(ClientId=client_id, ChallengeName="MFA_SETUP", Session=v["Session"],
                                      ChallengeResponses={"USERNAME": email})
    tokens = r.get("AuthenticationResult") or {}
    if tokens.get("RefreshToken"):
        idp.revoke_token(Token=tokens["RefreshToken"], ClientId=client_id)
    idp.admin_set_user_mfa_preference(UserPoolId=pool, Username=email,
                                      SoftwareTokenMfaSettings={"Enabled": True, "PreferredMfa": True})
    return secret


def fresh_code(pyotp, secret: str) -> str:
    """A TOTP code of a time step no earlier code of this run used (the pool refuses a code twice)."""
    totp = pyotp.TOTP(secret)
    step = int(time.time()) // 30
    while int(time.time()) // 30 == step:
        time.sleep(1)
    return totp.now()


def sign_in(page, base: str, email: str, password: str, pyotp, secret: str, timeout_s: int = 120) -> None:
    """Through the managed login (e-mail, password, TOTP: on one page or several) back to the distribution."""
    host = urllib.parse.urlsplit(base).netloc
    page.goto(base + "/", wait_until="domcontentloaded")
    deadline, last, same = time.time() + timeout_s, None, 0
    code_sel = ('input[autocomplete="one-time-code"], input[name*="code" i], input[id*="code" i], '
                'input[name*="totp" i], input[inputmode="numeric"]')
    while time.time() < deadline:
        page.wait_for_load_state("domcontentloaded")
        u = urllib.parse.urlsplit(page.url)
        if u.netloc == host and not u.path.startswith("/_auth/"):
            return
        visible = lambda sel: [e for e in page.locator(sel).all() if e.is_visible()]  # noqa: E731
        pw, code = visible('input[type="password"]'), visible(code_sel)
        user = visible('input[type="email"], input[name="username"], input[name="email"], input[autocomplete="username"]')
        if code and not pw:
            step, el = "code", code[0]
            el.fill(fresh_code(pyotp, secret))
        elif pw:
            step, el = "password", pw[0]
            if user and not user[0].input_value():
                user[0].fill(email)
            el.fill(password)
        elif user:
            step, el = "user", user[0]
            el.fill(email)
        else:
            page.wait_for_timeout(1000)
            continue
        same = same + 1 if step == last else 0
        if same >= 2:
            text = re.sub(r"\s+", " ", page.locator("body").inner_text())[:300]
            raise SmokeError(f"sign-in stuck at the {step} step on {u.netloc}{u.path}: {text}")
        last = step
        el.press("Enter")
        page.wait_for_timeout(1500)
    raise SmokeError(f"no sign-in within {timeout_s} s (at {urllib.parse.urlsplit(page.url).netloc})")


FETCH_JS = """async ([url, method, body, type]) => {
  const init = { method, credentials: 'same-origin', headers: {} };
  if (body !== null) { init.body = body; init.headers['content-type'] = type; }
  const r = await fetch(url, init);
  return { status: r.status, text: (await r.text()).slice(0, 4000) };
}"""

WS_JS = """(url) => new Promise((resolve) => {
  const ws = new WebSocket(url);
  const t = setTimeout(() => { try { ws.close(); } catch (e) {} resolve('timeout'); }, 20000);
  ws.onopen = () => { clearTimeout(t); ws.close(1000); resolve('open'); };
  ws.onerror = () => { clearTimeout(t); resolve('error'); };
  ws.onclose = (e) => { clearTimeout(t); resolve('closed ' + e.code); };
})"""


def fetch(page, path: str, method: str = "GET", body: str | None = None, ctype: str = "application/octet-stream") -> tuple[int, str]:
    r = page.evaluate(FETCH_JS, [path, method, body, ctype])
    return r["status"], r["text"]


def expect(status: int, want, what: str, text: str = "") -> None:
    if status not in (want if isinstance(want, tuple) else (want,)):
        # CloudFront's own error page: quote its reason and request id, not the HTML head.
        if "<TITLE>ERROR" in text:
            import re
            flat = " ".join(re.sub(r"<[^>]+>", " ", text.split("</HEAD>")[-1]).split())
            why = flat.split("Request ID:")[0].replace("503 ERROR", "").replace("502 ERROR", "").strip()
            rid = re.search(r"Request ID:\s*([\w=-]+)", text)
            raise SmokeError(f"{what}: {status} from CloudFront: {why[:300]!r} (request id {rid.group(1) if rid else '?'})")
        raise SmokeError(f"{what}: {status} {text[:200]!r}")


def check_signed_in(page, base: str, run_id: str) -> None:
    # The first request through a fresh VPC origin can get a 5xx from CloudFront (seen once, 503 with an empty body):
    # retried for up to 60 s, and reported when it happened, so a lasting 5xx still fails.
    for attempt in range(7):
        st, _ = fetch(page, "/")
        if st < 500 or attempt == 6:
            break
        print(f"note  GET / signed in: {st}, retrying in 10 s", flush=True)
        time.sleep(10)
    expect(st, 200, "GET / signed in")
    st, text = fetch(page, HEALTH)
    expect(st, 200, f"GET {HEALTH}", text)
    ok("signed in: 200 on / and /api/v1/health")
    wss = "wss://" + urllib.parse.urlsplit(base).netloc
    # The release reports ready when the backend serves HTTP; its WebSocket backends (Zenoh's remote-api plugin, the
    # chat agent) can take a minute more (test installs rf-new3 and rf-old3: /zenoh, then /chat failed ~70 s after
    # the backend started, both opened at once later). Up to 8 tries, 10 s apart, before it counts.
    for path in ("/zenoh", "/chat"):
        for attempt in range(1, WS_TRIES + 1):
            r = page.evaluate(WS_JS, wss + path)
            if r == "open":
                break
            if attempt == WS_TRIES:
                raise SmokeError(f"WebSocket {path}: {r} ({WS_TRIES} tries)")
            print(f"      WebSocket {path}: {r}, try {attempt} of {WS_TRIES}: again in {WS_RETRY_S} s", flush=True)
            time.sleep(WS_RETRY_S)
    ok("WebSocket upgrades: /zenoh and /chat open")
    st, text = fetch(page, "/api/v1/storage")
    expect(st, 200, "GET /api/v1/storage", text)
    info = json.loads(text)
    if info.get("mode") != "local":
        raise SmokeError(f"storage mode {info.get('mode')!r}: the public deployment keeps its data on the instance (local)")
    if not (info.get("roles") or {}).get("catalogue_admin"):
        raise SmokeError("the signed-in user is no catalogue admin (FACTORY_OS_CATALOGUE_ADMINS=default)")
    # User keys must start with one of the user's folders (products/, cells/, catalogue/: storage/local.rs Layout);
    # a dot-file at a folder's root makes no product, cell or catalogue folder and is deleted right after.
    for scope, rel, what in (("user", f"products/.fxt-smoke-{run_id}.txt", "upload"),
                             ("catalogue", f".fxt-smoke-{run_id}.txt", "catalogue write")):
        key = f"/api/v1/storage/object/{scope}/{rel}"
        data = f"factoryos smoke {run_id} {scope}\n"
        st, text = fetch(page, key, "PUT", data)
        expect(st, (200, 201, 204), f"{what} PUT", text)
        st, text = fetch(page, key)
        expect(st, 200, f"{what} read back", text)
        if text != data:
            raise SmokeError(f"{what}: read back other bytes")
        st, text = fetch(page, key, "DELETE")
        expect(st, (200, 204), f"{what} DELETE", text)
        ok(f"{what}: {scope} scope {rel} written, read back, deleted")
    st, text = fetch(page, "/api/v1/agentic/status")
    expect(st, 200, "GET /api/v1/agentic/status", text)
    ok(f"agentic status through the edge: {json.loads(text).get('state')}")


def start_sim(page, cleanup: Cleanup, timeout_s: int):
    """Start Simulation (unless the stack runs already) until ready; returns the stop to call while signed in (None
    when it ran before this test: then it is left running)."""
    st, text = fetch(page, "/api/v1/agentic/status")
    expect(st, 200, "agentic status", text)
    stop = None
    if json.loads(text).get("state") == "stopped":
        st, text = fetch(page, "/api/v1/agentic/start", "POST", "{}", "application/json")
        expect(st, 200, "Start Simulation", text)
        done = []

        def stop() -> None:
            if not done:
                done.append(True)
                fetch(page, "/api/v1/agentic/stop", "POST", "{}", "application/json")

        cleanup.add("Stop Simulation", stop)
    deadline = time.time() + timeout_s
    while True:
        s = json.loads(fetch(page, "/api/v1/agentic/status")[1])
        if s.get("state") == "ready":
            ok("Start Simulation: the planner stack is ready")
            return stop
        if s.get("state") == "failed" or time.time() > deadline:
            raise SmokeError(f"Start Simulation: {s.get('state')}: {s.get('error')}")
        time.sleep(10)


def live(a) -> int:
    import boto3
    import pyotp
    from playwright.sync_api import sync_playwright

    session = boto3.Session(profile_name=a.profile, region_name=a.region)
    region = session.region_name
    if not region:
        raise SmokeError("no region: --region or AWS_REGION")
    cfn, idp, ssm = session.client("cloudformation"), session.client("cognito-idp"), session.client("ssm")
    out = stack_outputs(cfn, a.stack_name)
    base = out["Url"].rstrip("/")
    pool, client_id, iid = out["UserPoolId"], out["UserPoolClientId"], out["InstanceId"]
    cognito_domain = urllib.parse.urlsplit(out["UserPoolDomain"]).netloc or out["UserPoolDomain"]
    say(f"==> {a.stack_name} in {region}: {base}/ (pool {pool}, instance {iid})")
    cleanup = Cleanup()
    try:
        props = {"client_id": client_id, "cognito_domain": cognito_domain,
                 "issuer": f"https://cognito-idp.{region}.amazonaws.com/{pool}", "region": region}
        n = check_denial(base, props)
        ok(f"denial matrix: {n} anonymous requests denied")
        n = sweep(idp, pool)
        ok(f"sweep: {n} smoke users older than 1 h deleted")
        if a.sweep_only:
            return 0

        run_id = str(uuid.uuid4())
        email = f"smoke-{run_id}@{SMOKE_DOMAIN}"
        password = make_password()
        cleanup.add(f"delete {email}", lambda: idp.admin_delete_user(UserPoolId=pool, Username=email), first=True)
        secret = create_user(idp, pool, client_id, email, password, pyotp)
        ok(f"user {email}: permanent password, TOTP enrolled")

        pw = sync_playwright().start()
        cleanup.add("stop Playwright", pw.stop)
        browser = pw.chromium.launch(headless=not a.headed)
        cleanup.add("close the browser", browser.close)
        # A fresh context: no storage state, no HAR, no tracing, no video (nothing of the sign-in is written).
        context = browser.new_context()
        page = context.new_page()
        page.set_default_timeout(30000)
        sign_in(page, base, email, password, pyotp, secret)
        names = {c["name"] for c in context.cookies()}
        if not {ID_COOKIE, ACCESS_COOKIE} <= names:
            raise SmokeError("signed in, but without the gate's session cookies")
        ok("sign-in through the managed login (password + TOTP): the gate's session cookies are set")

        check_signed_in(page, base, run_id)
        check_mcp(ssm, iid)
        check_router(ssm, iid)
        check_jam(ssm, iid, a.jam_cell)
        check_listeners(ssm, iid, "after sign-in")
        if not a.skip_sim:
            stop = start_sim(page, cleanup, a.sim_timeout)
            check_listeners(ssm, iid, "after Start Simulation")
            if stop:
                stop()
                ok("Stop Simulation")

        page.goto(base + LOGOUT_PATH, wait_until="domcontentloaded")
        left = {c["name"] for c in context.cookies(base + "/")} & {ID_COOKIE, ACCESS_COOKIE, "__Host-fos_refresh"}
        if left:
            raise SmokeError(f"signed out, but {', '.join(sorted(left))} still set")
        # The context's own cookie jar (whatever it still holds), no redirect followed.
        st = context.request.get(base + HEALTH, max_redirects=0).status
        expect(st, 401, "the API after sign-out")
        ok("sign-out: the session cookies are cleared, the API is 401 again")
        say("==> smoke test OK")
        return 0
    finally:
        cleanup.run()


# --- --self-test: offline, a stub gate on 127.0.0.1 ---------------------------------------------------------------
class StubGate(BaseHTTPRequestHandler):
    """The gate's anonymous answers (lambda/edge-auth/index.mjs), or a broken one (`mode`)."""

    protocol_version = "HTTP/1.1"
    # The stub pool's domain: a reserved name (RFC 2606), not a Cognito domain the export's scan would flag.
    mode, client_id, cognito_domain = "good", "stubclient", "login.example.invalid"

    def log_message(self, *args) -> None:
        pass

    def _answer(self) -> None:
        base = f"http://{self.headers.get('host')}"
        path = urllib.parse.urlsplit(self.path).path
        upgrade = (self.headers.get("upgrade") or "").lower() == "websocket"
        cookie = self.headers.get("cookie") or ""
        if self.mode == "leaky" and path == HEALTH and "eyJhbGciOiJub25l" in cookie:  # an alg=none token's header
            return self._send(200, {"content-type": "application/json"}, b'{"ok":true}')
        if path.startswith("/_auth/"):
            return self._send(400, {}, b"bad request\n")
        if upgrade and self.headers.get("origin") != base:
            return self._send(403, {}, b"Forbidden\n")
        accept = self.headers.get("accept") or ""
        mode = self.headers.get("sec-fetch-mode")
        nav = self.command == "GET" and not upgrade and (mode == "navigate" if mode else
                                                         not path.startswith("/api/") and ("text/html" in accept or not accept))
        if nav:
            client = "otherclient" if self.mode == "wrong-client" else self.client_id
            q = urllib.parse.urlencode({"client_id": client, "redirect_uri": f"{base}{CALLBACK_PATH}",
                                        "response_type": "code", "code_challenge_method": "S256",
                                        "code_challenge": "x", "state": "s"})
            return self._send(302, {"location": f"https://{self.cognito_domain}/oauth2/authorize?{q}"}, b"")
        return self._send(401, {"www-authenticate": f'Bearer {REALM}', "content-type": "application/json"},
                          b'{"error":"unauthorized"}')

    def _send(self, status: int, headers: dict, body: bytes) -> None:
        self.send_response(status)
        for k, v in headers.items():
            self.send_header(k, v)
        self.send_header("content-length", str(len(body)))
        self.send_header("connection", "close")  # one request per connection, as http_request makes them
        self.close_connection = True
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(body)

    do_GET = do_POST = do_HEAD = _answer


def self_test() -> int:
    failures = 0

    def check(name: str, cond: bool) -> None:
        nonlocal failures
        say(f"{'ok  ' if cond else 'FAIL'}  {name}")
        failures += 0 if cond else 1

    props = {"client_id": StubGate.client_id, "cognito_domain": StubGate.cognito_domain,
             "issuer": "https://cognito-idp.eu-central-1.amazonaws.com/eu-central-1_stub", "region": "eu-central-1"}
    for mode in ("good", "leaky", "wrong-client"):
        handler = type(f"Stub_{mode}", (StubGate,), {"mode": mode})
        server = ThreadingHTTPServer(("127.0.0.1", 0), handler)  # loopback only, an ephemeral port
        host, port = server.server_address[:2]
        check(f"stub gate ({mode}) listens on loopback only", host == "127.0.0.1")
        t = threading.Thread(target=server.serve_forever, daemon=True)
        t.start()
        try:
            base = f"http://127.0.0.1:{port}"
            fatal, problems, n = run_matrix(base, props)
            if mode == "good":
                check(f"denial matrix passes on a correct gate ({n} checks)", fatal is None and not problems and n == 16)
                check("check_denial returns the number of checks", check_denial(base, props, rounds=1) == 16)
            elif mode == "leaky":
                check("denial matrix: a 2xx for an alg=none token is fatal", fatal is not None and "alg-none" in fatal)
                try:
                    check_denial(base, props, rounds=1)
                    check("check_denial raises on a leaky gate", False)
                except SmokeError as e:
                    check("check_denial raises on a leaky gate", "let an anonymous request through" in str(e))
            else:
                check("denial matrix: a login redirect with another client_id is a problem",
                      fatal is None and any("another client_id" in p for p in problems))
        finally:
            server.shutdown()
            server.server_close()
    try:
        socket.create_connection(("127.0.0.1", port), timeout=1).close()
        check("stub gate is gone after the test", False)
    except OSError:
        check("stub gate is gone after the test", True)

    toks = forged_tokens(props["issuer"], props["client_id"], "eu-central-1", now=1_700_000_000)
    exp = json.loads(base64.urlsafe_b64decode(toks["expired"]["access"].split(".")[1] + "=="))
    check("forged tokens: expired is expired, alg-none unsigned",
          exp["exp"] < 1_700_000_000 and toks["alg-none"]["id"].endswith("."))

    # SRP: the client's S equals the server's for the same password, salt and ephemerals (RFC 5054 arithmetic).
    pool_name, user_id, password, salt = "abcDEF123", "11111111-2222-3333-4444-555555555555", make_password(), "a1b2c3d4" * 4
    x = srp_x(pool_name, user_id, password, salt)
    v = pow(G, x, N)
    a, b = secrets.randbelow(N - 2) + 1, secrets.randbelow(N - 2) + 1
    big_a, big_b = srp_a(a), (K * v + pow(G, b, N)) % N
    u = srp_u(big_a, big_b)
    check("SRP: client and server derive the same session key",
          srp_client_s(a, big_b, x, u) == pow(big_a * pow(v, u, N), b, N))
    check("SRP: another password derives another key",
          srp_client_s(a, big_b, srp_x(pool_name, user_id, password + "x", salt), u) != pow(big_a * pow(v, u, N), b, N))
    check("SRP: pad_hex as Cognito hashes", pad_hex(0x8f) == "008f" and pad_hex(0xf) == "0f" and pad_hex(0x7f) == "7f")
    claim = srp_claim("eu-central-1_" + pool_name, a, big_a, {"USER_ID_FOR_SRP": user_id, "SRP_B": format(big_b, "x"),
                                                              "SALT": salt, "SECRET_BLOCK": base64.b64encode(b"blk").decode()},
                      password, dt.datetime(2026, 11, 3, 9, 5, 1, tzinfo=dt.timezone.utc))
    check("SRP: the claim's timestamp has no leading zero in the day", claim["TIMESTAMP"] == "Tue Nov 3 09:05:01 UTC 2026")
    check("SRP: the claim is signed (HMAC-SHA256)", len(base64.b64decode(claim["PASSWORD_CLAIM_SIGNATURE"])) == 32)

    order = []
    c = Cleanup.__new__(Cleanup)  # no atexit / signal handlers in the self-test
    c._steps, c._lock = [], threading.Lock()
    for name, first in (("browser", False), ("user", True), ("sim", False)):
        c.add(name, lambda n=name: order.append(n), first=first)
    c.run()
    c.run()
    check("cleanup: the user first, then last-in first-out, once", order == ["user", "sim", "browser"])

    pw = make_password()
    check("password: 32 characters, lower, upper, digit and symbol",
          len(pw) == 32 and any(c.islower() for c in pw) and any(c.isupper() for c in pw) and any(c.isdigit() for c in pw)
          and any(not c.isalnum() for c in pw))

    any4 = ".".join("0" * 4)  # the IPv4 wildcard, in ss samples only (nothing here binds to it)
    good_ss = "\n".join([
        f"udp   UNCONN 0      0         127.0.0.54:53         {any4}:*",
        f"udp   UNCONN 0      0      127.0.0.53%lo:53         {any4}:*",
        f"udp   UNCONN 0      0     10.0.1.23%ens5:68         {any4}:*",
        f"udp   UNCONN 0      0          127.0.0.1:323        {any4}:*",
        f"udp   UNCONN 0      0              [::1]:323           [::]:*",
        f"udp   UNCONN 0      0 [fe80::8ff:1%ens5]:546           [::]:*",
        f"tcp   LISTEN 0      4096       127.0.0.1:8893       {any4}:*",
        f"tcp   LISTEN 0      4096       127.0.0.1:7447       {any4}:*",
        f"tcp   LISTEN 0      4096         {any4}:80         {any4}:*",
        f"tcp   LISTEN 0      4096   127.0.0.53%lo:53         {any4}:*",
        f"tcp   LISTEN 0      128   [::ffff:127.0.0.1]:8890     *:*",
    ])
    bad, n = listener_problems(good_ss)
    check(f"listeners: loopback, :80 and the DHCP clients pass ({n} sockets)", bad == [] and n == 11)
    bad, _ = listener_problems(good_ss + f"\ntcp   LISTEN 0      128          {any4}:8890       {any4}:*  users:((\"python\",pid=1,fd=3))")
    check("listeners: the planner MCP on the wildcard address :8890 fails", len(bad) == 1 and f"{any4}:8890" in bad[0])
    bad, _ = listener_problems(f"tcp LISTEN 0 128 *:22 *:*\nudp UNCONN 0 0 {any4}:68 {any4}:*\ntcp LISTEN 0 128 [::]:443 [::]:*")
    check("listeners: sshd, an unbound UDP :68 and :443 fail", len(bad) == 3)

    now = dt.datetime(2026, 11, 12, 12, 0, tzinfo=dt.timezone.utc)

    def user(email: str, age_min: int) -> dict:
        return {"Username": f"u-{email}", "Attributes": [{"Name": "email", "Value": email}],
                "UserCreateDate": now - dt.timedelta(minutes=age_min)}

    old_smoke = f"smoke-{uuid.uuid4()}@example.com"
    picked = sweep_candidates([user(old_smoke, 61), user(f"smoke-{uuid.uuid4()}@example.com", 30),
                               user("smoke-team@corp.example", 600), user("alice@example.com", 600),
                               user(f"smoke-{uuid.uuid4()}@example.com.evil", 600)], now)
    check("sweep: only smoke-<uuid>@example.com users older than 1 h", picked == [f"u-{old_smoke}"])

    # umarell_jam's verdict (JAM_SCRIPT's report): skipped only when not shipped or off; any missing part fails.
    def described(warnings=(), available=None, name="umarell_jam", is_error=False) -> str:
        e = {"name": name, "warnings": list(warnings)}
        if available is not None:
            e["available"] = available
        return json.dumps({"jsonrpc": "2.0", "id": 1, "result": {"content": [{"type": "text", "text": json.dumps(e)}],
                                                                 "isError": is_error}})
    good = {"shipped": True, "off": False, "incomplete": False, "source": "/var/lib/factoryos/umarell-76ea13a",
            "runtime": "", "setup_rc": 0, "setup_tail": "", "describe": described()}
    check("jam: set up -> ok", jam_verdict(good)[0] == "ok")
    check("jam: not shipped -> skip", jam_verdict({"shipped": False})[0] == "skip")
    check("jam: off on the host -> skip", jam_verdict({**good, "off": True})[0] == "skip")
    v = jam_verdict({**good, "source": "", "incomplete": True})
    check("jam: no plain umarell -> fail, provision-incomplete named", v[0] == "fail" and "no plain umarell" in v[1]
          and "provision-incomplete" in v[1])
    v = jam_verdict({**good, "runtime": "ModuleNotFoundError: No module named 'toppra'"})
    check("jam: no toppra -> fail (numerical runtime is incomplete)", v[0] == "fail" and "toppra" in v[1]
          and "numerical runtime is incomplete" in v[1])
    v = jam_verdict({**good, "setup_rc": 124, "setup_tail": "timed out after 180 s"})
    check("jam: setup timed out -> fail", v[0] == "fail" and "exited 124" in v[1] and "timed out" in v[1])
    check("jam: setup ran no more (a crash before it) -> fail", jam_verdict({**good, "setup_rc": None})[0] == "fail")
    v = jam_verdict({**good, "describe": described(["no plain umarell 76ea13a on this host (/var/lib/factoryos/umarell-76ea13a)"])})
    check("jam: describe_planner's 'no plain umarell' warning -> fail", v[0] == "fail" and "warns" in v[1])
    v = jam_verdict({**good, "describe": described(["umarell_jam's numerical runtime is incomplete in /x (toppra)"])})
    check("jam: describe_planner's runtime warning -> fail", v[0] == "fail")
    v = jam_verdict({**good, "describe": described(["umarell_jam's component cache on this workspace is empty"])})
    check("jam: an empty component cache is no setup failure -> ok", v[0] == "ok")
    check("jam: describe_planner unanswered -> fail", jam_verdict({**good, "describe": "", "describe_error": "curl: (7)"})[0] == "fail")
    check("jam: describe_planner of an unknown planner -> fail",
          jam_verdict({**good, "describe": described(name=None, is_error=True)})[0] == "fail")
    v = jam_verdict({**good, "describe": described(available=False)}, "body_weld_3")
    check("jam: --jam-cell, not available there -> fail", v[0] == "fail" and "body_weld_3" in v[1])
    check("jam: --jam-cell, available -> ok", jam_verdict({**good, "describe": described(available=True)}, "c")[0] == "ok")
    import ast

    def consts(cell):
        body = jam_script(cell).split("<<'FOS_JAM_PY'\n", 1)[1].rsplit("FOS_JAM_PY", 1)[0]
        for node in ast.walk(ast.parse(body)):
            if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Tuple) \
                    and [getattr(t, "id", "") for t in node.targets[0].elts] == ["umarell", "setup_s", "cell"]:
                return [ast.literal_eval(v) for v in node.value.elts]
        return None
    check("jam: the instance's script parses, its placeholders python literals",
          consts(None) == ["/var/lib/factoryos/umarell", JAM_SETUP_S, None]
          and consts("body_weld_3") == ["/var/lib/factoryos/umarell", JAM_SETUP_S, "body_weld_3"])
    try:
        jam_script("x'; rm -rf /")
        check("jam: a --jam-cell that is no cell name is refused", False)
    except SmokeError:
        check("jam: a --jam-cell that is no cell name is refused", True)

    try:
        import pyotp
    except ImportError:
        say("skip  TOTP (pyotp is not installed: pip install -r deployment/test/requirements-smoke.txt)")
    else:
        secret = pyotp.random_base32()
        check("TOTP: pyotp codes verify", pyotp.TOTP(secret).verify(pyotp.TOTP(secret).now()))

    say(f"{'self-test OK' if not failures else f'self-test FAILED: {failures}'}")
    return 1 if failures else 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="The post-install smoke test of a FactoryOS public deployment (see the module docstring).",
        epilog="Exit 0: every check passed; 1: a check failed (the cleanup still ran); 2: usage.")
    ap.add_argument("--stack-name", default="factoryos", help="the main stack's name (install.sh --stack-name; default factoryos)")
    ap.add_argument("--region", default=os.environ.get("AWS_REGION") or os.environ.get("AWS_DEFAULT_REGION"),
                    help="the main stack's region (default: AWS_REGION, then AWS_DEFAULT_REGION)")
    ap.add_argument("--profile", default=None, help="the AWS profile (default: the environment's credentials)")
    ap.add_argument("--skip-sim", action="store_true", help="no Start Simulation (and no second listener check)")
    ap.add_argument("--sim-timeout", type=int, default=900, help="seconds to wait for Start Simulation (default 900)")
    ap.add_argument("--headed", action="store_true", help="show the browser (debugging the sign-in)")
    ap.add_argument("--jam-cell", default=None,
                    help="describe_planner umarell_jam for this cell (it needs a task program): available, no setup "
                         "warnings (default: without a cell)")
    ap.add_argument("--sweep-only", action="store_true",
                    help="only the denial matrix and the sweep of smoke users older than 1 h")
    ap.add_argument("--self-test", action="store_true",
                    help="the offline checks against a stub gate on 127.0.0.1 (no AWS, no browser)")
    a = ap.parse_args(argv)
    if a.self_test:
        return self_test()
    try:
        return live(a)
    except SmokeError as e:
        say(f"FAIL  {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
