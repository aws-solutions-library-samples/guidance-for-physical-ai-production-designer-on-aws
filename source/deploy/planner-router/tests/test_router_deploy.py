"""The planner router's deployability tools (check_deployability, fix_deployability) against a fake backend:

    python3 -I -m unittest discover -s deploy/planner-router/tests
"""
from __future__ import annotations

import json
import sys
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import server as R  # noqa: E402
from test_router import RouterBase  # noqa: E402

# The backend's report for scene_fit's export of 2026-10-16 (gateway 0.0.0.0, safety module on the KUKA VCs).
SCENE_FIT = {
    "cell": "scene_fit", "build": "20261016T174822044Z-bt_1792156523_scene_fit", "planId": "16#A1BE",
    "profile": "vw-ks24", "ok": False, "fails": 2, "warns": 0, "target": "the virtual cell (2 KUKA VC(s))",
    "rules": [
        {"id": "export.files", "title": "Export complete", "scope": "export", "status": "pass", "message": "ok"},
        {"id": "cell.safety", "title": "Safety settings the KUKA VCs can satisfy", "scope": "cell", "status": "fail",
         "message": "robot1: a safety module: Start production will fail with ERR_SAFETY_TIMEOUT",
         "hint": "Hardware Configuration: robot1 → Safety module off; then Export again", "robots": ["robot1"],
         "fixes": [{"kind": "hardware", "field": "robots.robot1.safetyModule", "value": False}]},
        {"id": "cell.network", "title": "Head PLC network", "scope": "cell", "status": "fail",
         "message": "gateway none (0.0.0.0)", "hint": "Hardware Configuration: Network → Gateway 192.0.2.1",
         "fixes": [{"kind": "hardware", "field": "network.gateway", "value": "192.0.2.1"}]},
        {"id": "krl.names", "title": "KRL names known to the controllers", "scope": "export", "status": "pass",
         "message": "every name is declared"},
    ],
}
FIXED = {**SCENE_FIT, "build": "20261016T190000000Z-bt_1792156523_scene_fit", "ok": True, "fails": 0,
         "rules": [dict(r, status="pass", fixes=[]) for r in SCENE_FIT["rules"]]}


class FakeBackend:
    def __init__(self):
        self.calls = []
        outer = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _send(self, code, text, kind="application/json"):
                data = text.encode()
                self.send_response(code)
                self.send_header("Content-Type", kind)
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
                outer.calls.append((self.path, body))
                if self.path == "/api/v1/infra/deploy/check":
                    if body.get("build", "").endswith("-nope"):
                        return self._send(404, "no successful Control Code Export x in cell scene_fit", "text/plain")
                    return self._send(200, json.dumps(SCENE_FIT))
                if self.path == "/api/v1/infra/deploy/fix":
                    return self._send(200, json.dumps({
                        "changes": ["Hardware Configuration: robot1 Safety module → off",
                                    "Hardware Configuration: Network Gateway → 192.0.2.1",
                                    "Export agentic_planner/plans/bt_1792156523_scene_fit.json again with the new settings"],
                        "backup": "/c/plan2cell/hardware.json.bak-1792170000", "build": FIXED["build"],
                        "before": SCENE_FIT, "report": FIXED}))
                return self._send(404, "no route", "text/plain")

        self.httpd = ThreadingHTTPServer(("127.0.0.1", 0), H)
        self.url = f"http://127.0.0.1:{self.httpd.server_address[1]}"
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def close(self):
        self.httpd.shutdown()
        self.httpd.server_close()


class DeployToolsTest(RouterBase):
    def setUp(self):
        super().setUp()
        self.backend = FakeBackend()
        self.router.s.backend_url = self.backend.url

    def tearDown(self):
        self.backend.close()
        super().tearDown()

    def call(self, name, **arguments):
        code, r, _ = R.handle_rpc(self.router, {"jsonrpc": "2.0", "id": 1, "method": "tools/call",
                                                "params": {"name": name, "arguments": arguments}})
        self.assertEqual(code, 200)
        return r["result"]["isError"], json.loads(r["result"]["content"][0]["text"])

    def test_tools_are_listed(self):
        _, r, _ = R.handle_rpc(self.router, {"jsonrpc": "2.0", "id": 2, "method": "tools/list"})
        tools = {t["name"]: t for t in r["result"]["tools"]}
        self.assertEqual(set(tools["check_deployability"]["inputSchema"]["properties"]), {"cell", "build"})
        self.assertEqual(set(tools["fix_deployability"]["inputSchema"]["properties"]), {"cell", "build", "rules"})
        self.assertIn("consent", tools["fix_deployability"]["description"])

    def test_check_returns_the_table_with_the_fixes(self):
        error, result = self.call("check_deployability", cell="scene_fit", build=SCENE_FIT["build"])
        self.assertFalse(error, result)
        self.assertEqual(self.backend.calls, [("/api/v1/infra/deploy/check", {"cell": "scene_fit", "build": SCENE_FIT["build"]})])
        self.assertFalse(result["deployable"])
        table = result["table"]
        self.assertIn("NOT deployable: 2 problem(s), 0 warning(s)", table)
        self.assertIn("✗ Safety settings the KUKA VCs can satisfy [cell.safety]: robot1: a safety module", table)
        self.assertIn("auto-fix: Hardware Configuration network.gateway = \"192.0.2.1\"", table)
        self.assertIn("✓ KRL names known to the controllers [krl.names]", table)
        # The newest export of the active cell: no body.
        error, _ = self.call("check_deployability")
        self.assertFalse(error)
        self.assertEqual(self.backend.calls[-1], ("/api/v1/infra/deploy/check", {}))

    def test_fix_reports_the_changes_and_the_new_export(self):
        error, result = self.call("fix_deployability", cell="scene_fit", rules=["cell.safety", "cell.network"])
        self.assertFalse(error, result)
        self.assertEqual(self.backend.calls[-1], ("/api/v1/infra/deploy/fix", {"cell": "scene_fit",
                                                                               "rules": ["cell.safety", "cell.network"]}))
        self.assertTrue(result["deployable"])
        self.assertEqual(result["new_build"], FIXED["build"])
        self.assertEqual(len(result["changes"]), 3)
        self.assertIn("deployable", result["table"])

    def test_bad_arguments_and_backend_errors_are_tool_errors(self):
        for name, args in (("check_deployability", {"cell": "../x"}), ("check_deployability", {"build": "latest; rm"}),
                           ("fix_deployability", {"rules": "cell.safety"}), ("fix_deployability", {"rules": ["DROP"]})):
            error, result = self.call(name, **args)
            self.assertTrue(error, args)
            self.assertIn("error", result)
        self.assertEqual(self.backend.calls, [])
        error, result = self.call("check_deployability", build="20261016T174822044Z-nope")
        self.assertTrue(error)
        self.assertIn("no successful Control Code Export", result["error"])


if __name__ == "__main__":
    unittest.main()
