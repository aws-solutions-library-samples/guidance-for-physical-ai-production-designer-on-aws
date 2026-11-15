"""Unit tests of the planner router (standard library only):

    python3 -m unittest discover -s deploy/planner-router/tests -t deploy/planner-router

A fake factoryos-cell-planner (tools/btexport/__main__.py writes a fixture plan and report; its exit code
and delay come from the --from spec) and a fake umarell MCP server on a free port stand in for the
real ones.
"""
from __future__ import annotations

import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ROUTER_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROUTER_DIR))
import server as R  # noqa: E402

# The Robo Fitter's tools, when this server has them (they are added alongside).
R_FITTER_TOOLS = {t["name"] for t in R.TOOLS} & {"check_reachability", "suggest_layout"}

FAKE_BTEXPORT = r'''
import argparse, json, os, sys, time
from pathlib import Path
ap = argparse.ArgumentParser()
for a in ("--from", "--profile", "--cell", "--tree-id", "--out", "--work", "--report", "--repo", "--home-tail",
          "--home-tol"):
    ap.add_argument(a)
ap.add_argument("--dense-check", action="store_true")
a = ap.parse_args()
spec = getattr(a, "from")
exit_code = 0
for part in spec.split(","):
    if part.startswith("exit="):
        exit_code = int(part[5:])
    if part.startswith("sleep="):
        time.sleep(float(part[6:]))
# the planner reads its cell from UMARELL_REPO/cells/<cell> (the router's view)
assert (Path(os.environ["UMARELL_REPO"]) / "cells" / a.cell / "agentic_planner" / "config" / "robots.yaml").exists()
assert (Path(os.environ["UMARELL_REPO"]) / "cells" / a.cell / "robots" / "r" / "meshes").is_dir()
print(f"[btexport] {spec} -> {a.out}: profile {a.profile}", flush=True)
if exit_code == 2:
    print("[btexport] unsupported: bad spec", file=sys.stderr)
    sys.exit(2)
out = Path(a.out)
plan = {"tree_id": a.tree_id, "cell_id": a.cell, "created_s": time.time(), "status": "ok", "num_operations": 3,
        "behaviour_tree": {"tree_id": a.tree_id, "cell_id": a.cell, "root": {"type": "Sequence", "children": []}}}
out.write_text(json.dumps(plan))
out.with_suffix(".xml").write_text("<root/>")
if "crash" in spec.split(","):     # as D.plan_check raising after C.export wrote the plan: Python exits 1
    raise RuntimeError("dense check blew up")
Path(a.report).write_text(json.dumps({"export_pass": exit_code == 0, "timing": {"executor_end_s": 53.76,
    "makespan_motion_s": 52.76}, "checks": {"limits": {"pass": exit_code == 0}}, "dense": {"valid": True,
    "sphere_min_m": 0.27}, "warnings": []}))
print("[btexport] URDF limits: ...", flush=True)
sys.exit(exit_code)
'''

FAKE_U3 = r'''
"""Stands in for factoryos-cell-planner's umarell_fast/u3 (python -m umarell_fast.u3)."""
import argparse, json, shutil, sys, time
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
SPEED_PRESETS = {"default": {}, "max": {"velocity_scale": 1.0}}     # U3's --speed (umarell_u3.py looks for it)


def working_copy(A, fixes):
    dest = ROOT / "data" / "umarell_fast" / "repo_U3_fix"
    if not dest.is_dir():
        shutil.copytree(ROOT / "third_party" / "sample-umarell", dest, ignore=shutil.ignore_patterns(".venv*"))
        print("[apply] patched copy", dest, flush=True)
    return dest, {"patches": [{"patch": "docs/upstream/patches/01_x.diff"}]}


def main(argv=None):
    ap = argparse.ArgumentParser()
    for o in ("--cell", "--tag", "--task", "--out", "--workers"):
        ap.add_argument(o)
    ap.add_argument("--with-upstream-fixes", action="store_true")
    ap.add_argument("--speed", choices=sorted(SPEED_PRESETS), default="default")
    a = ap.parse_args(argv)
    repo, _ = working_copy(None, True)
    cell = repo / "cells" / a.cell
    task = json.loads((cell / "agentic_planner" / "tasks" / a.task).read_text())
    ops = [o["id"] for o in task["operations"]]
    if any(o.startswith("SLOW") for o in ops):
        time.sleep(0.6)
    log = {"cell": a.cell, "task": a.task, "ops": ops, "own": (cell / ".factoryos-cell").is_file(),
           "robots": str((cell / "robots").resolve()), "speed": a.speed}
    (ROOT / "last_run.json").write_text(json.dumps(log))
    out = Path(a.out)
    (out / "plans").mkdir(parents=True, exist_ok=True)
    ok = "FAIL" not in ops
    res = {"success": ok, "wall_s": 1.5, "workers": 2, "kernels_requested": "u1", "patches": ["x/01_x.diff"],
           "legs": [{"kind": "weld", "status": "ok"} for _ in ops], "speed": {"preset": a.speed}}
    if ok:
        plan = out / "plans" / f"bt_1_{a.cell}.json"
        plan.write_text(json.dumps({"tree_id": "bt_1", "cell_id": a.cell, "status": "ok"}))
        res["plan_file"] = str(plan)
    else:
        res["unreachable"] = ["FAIL"]
    (out / f"{a.tag}.json").write_text(json.dumps(res))
    print("[u3] RESULT " + json.dumps({"success": ok, "wall_s": 1.5, "n_steps": 3, "welds": len(ops)}), flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
'''


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class FakeUmarell:
    """The umarell planner's /mcp (plan_trajectories, get_plan_status) and /health, and the backend's Prepare Cell."""

    def __init__(self):
        self.calls = []
        self.prepare = None     # the backend's Prepare Cell (also served here): a callable(cell), or None = fails
        self.prepare_calls = 0
        self.prepare_cells = []
        self.old_backend = False    # a backend whose Prepare Cell takes no cell (no prepareCell in its status)
        self.status = "done"        # what get_plan_status says
        self.plans = 0
        self.lock = threading.Lock()
        outer = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _json(self, value):
                data = json.dumps(value).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self):
                if self.path == "/api/v1/agentic/status":
                    st = {"prepare": "done" if outer.prepare else "failed",
                          "prepareError": None if outer.prepare else "no scene.yaml"}
                    if not outer.old_backend:
                        st["prepareCell"] = outer.prepare_cells[-1] if outer.prepare_cells else ""
                    return self._json(st)
                self.send_response(200 if self.path == "/health" else 404)
                self.send_header("Content-Length", "0")
                self.end_headers()

            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
                if self.path == "/api/v1/agentic/prepare":
                    outer.prepare_calls += 1
                    cell = None if outer.old_backend else body.get("cell")
                    outer.prepare_cells.append(cell or RouterBase.CID)
                    if outer.prepare:
                        outer.prepare(cell or RouterBase.CID)
                    if outer.old_backend:
                        return self._json({"prepare": "preparing"})
                    return self._json({"prepare": "preparing", "prepareCell": cell or RouterBase.CID})
                if self.path != "/mcp":     # the backend's other endpoints (agentic/start)
                    return self._json({})
                p = body["params"]
                outer.calls.append((p["name"], p["arguments"]))
                if p["name"] == "plan_trajectories":
                    with outer.lock:
                        n, outer.plans = outer.plans, outer.plans + 1
                    res = {"accepted": True, "tree_id": f"bt_{1700000000 + n}_{p['arguments']['cell_id']}",
                           "status": "planning", "model": "opus-5"}
                elif p["name"] == "cancel_planning":
                    res = {"cancelled": True}
                else:
                    res = {"status": outer.status, "success": outer.status == "done"}
                data = json.dumps({"jsonrpc": "2.0", "id": body["id"],
                                   "result": {"content": [{"type": "text", "text": json.dumps(res)}]}}).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

        self.httpd = ThreadingHTTPServer(("127.0.0.1", 0), H)
        self.url = f"http://127.0.0.1:{self.httpd.server_address[1]}/mcp"
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def close(self):
        self.httpd.shutdown()
        self.httpd.server_close()


class RouterBase(unittest.TestCase):
    CID = "welding_cell_2_small"

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="router-test-"))
        t = self.tmp
        # the cell planner
        src = t / "src"
        (src / "tools" / "btexport").mkdir(parents=True)
        (src / "tools" / "__init__.py").write_text("")
        (src / "tools" / "btexport" / "__init__.py").write_text("")
        (src / "tools" / "btexport" / "__main__.py").write_text(FAKE_BTEXPORT)
        for ck in ("runs/a.pt", "runs/b.pt"):
            (src / ck).parent.mkdir(parents=True, exist_ok=True)
            (src / ck).write_text("ckpt")
        (src / "trajgen" / "samples" / self.CID / "expected").mkdir(parents=True)
        (src / "trajgen" / "samples" / self.CID / "expected" / "plan.json").write_text("{}")
        (src / ".cell-planner.txt").write_text("abc1234  deadbeef  key\n")
        # the policies' weld IK table of the training cell (rl/hier/instances.table_path)
        (src / "data").mkdir()
        for cid in (self.CID, "other_cell", "jam2"):   # and of the other cells the tests plan with a policy
            (src / "data" / ("weld_ik.json" if cid == self.CID else f"weld_ik_{cid}.json")).write_text("{}")
        # the umarell checkout's training cell (no meshes, as in its git archive)
        uc = t / "umarell" / "cells" / self.CID
        for rel, text in (("agentic_planner/config/robots.yaml", "robots: 1\n"),
                          ("agentic_planner/config/obstacles.yaml", "obstacles: []\n"),
                          ("agentic_planner/tasks/car_door_spot_welding.json", '{"operations": [{"id": "W01"}]}'),
                          ("robots/r/r.urdf", "<robot/>"), ("scene.yaml", "scene: 1\n")):
            (uc / rel).parent.mkdir(parents=True, exist_ok=True)
            (uc / rel).write_text(text)
        # the FactoryOS cell: same scene, robots with meshes, no agentic_planner/config
        self.cell = t / "cells" / self.CID
        (self.cell / "robots" / "r" / "meshes").mkdir(parents=True)
        shutil.copy(uc / "robots/r/r.urdf", self.cell / "robots/r/r.urdf")
        shutil.copy(uc / "scene.yaml", self.cell / "scene.yaml")
        (t / "cells" / "other_cell").mkdir()
        (t / "cells" / "other_cell" / "scene.yaml").write_text("x\n")
        # config: the active cell
        self.config = t / "factoryos.config.json"
        self.config.write_text(json.dumps({"cellsDir": str(t / "cells"), "cellDir": str(self.cell)}))
        # the catalogue: the real planners.json, with fake checkpoints / specs and this cell's checksums
        cat = json.loads((ROUTER_DIR / "planners.json").read_text())
        cat["training_cells"] = {self.CID: {
            "agentic_planner/tasks/car_door_spot_welding.json": sha(uc / "agentic_planner/tasks/car_door_spot_welding.json"),
            "agentic_planner/config/robots.yaml": sha(uc / "agentic_planner/config/robots.yaml"),
            "agentic_planner/config/obstacles.yaml": sha(uc / "agentic_planner/config/obstacles.yaml"),
            "robots/r/r.urdf": sha(uc / "robots/r/r.urdf"),
            "scene.yaml": sha(uc / "scene.yaml")}}
        for p in cat["planners"]:
            if p["kind"] == "policy":
                p["checkpoint"] = "runs/a.pt"
                p["from"] = "fake:a"
        self.cat = cat
        self.planners = t / "planners.json"
        self.planners.write_text(json.dumps(cat))
        self.umarell = FakeUmarell()
        # Prepare Cell writes the cell's agentic_planner/cell.yaml (the umarell route needs it)
        def prepare(cell):
            marker = t / "cells" / cell / R.PREPARED_MARKER
            marker.parent.mkdir(parents=True, exist_ok=True)
            marker.write_text("name: x\n")
        self.umarell.prepare = prepare
        self.router = self.make_router()

    WORKERS = 4
    UMARELL_WORKERS = 8

    def make_router(self, workers=None, umarell_workers=None):
        s = R.Settings(cell_planner=self.tmp / "src", python=Path(sys.executable), umarell=self.tmp / "umarell",
                       config=self.config, work=self.tmp / "work", planners_file=self.planners,
                       umarell_url=self.umarell.url, backend_url=self.umarell.url.rsplit("/mcp", 1)[0],
                       workers=workers or self.WORKERS, umarell_workers=umarell_workers or self.UMARELL_WORKERS)
        r = R.Router(s)
        r.burst_mode = "off"      # never remote planning by the test host's CPU count (test_router_burst.py tests it)
        return r

    def other_cell(self, task="welds", prepared=True):
        """other_cell (not in the umarell checkout) with a task program, robots with meshes and Prepare Cell's
        config (its own view: the policies read its prepared agentic_planner/); prepared: and its cell.yaml."""
        other = self.tmp / "cells" / "other_cell"
        if prepared:
            (other / R.PREPARED_MARKER).parent.mkdir(parents=True, exist_ok=True)
            (other / R.PREPARED_MARKER).write_text("name: other_cell\n")
        for rel, text in (("agentic_planner/config/robots.yaml", "robots: 2\n"),
                          (f"{R.TASKS_DIR}/{task}.json", '{"operations": [{"id": "W01"}, {"id": "W02"}]}'),
                          ("robots/r/r.urdf", "<robot/>")):
            (other / rel).parent.mkdir(parents=True, exist_ok=True)
            (other / rel).write_text(text)
        (other / "robots" / "r" / "meshes").mkdir(exist_ok=True)
        return other

    def last_plan_call(self):
        return [c for c in self.umarell.calls if c[0] == "plan_trajectories"][-1]

    def tearDown(self):
        self.umarell.close()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def set_spec(self, planner, spec):
        self.router.planners[planner]["from"] = spec

    def wait(self, job_id, timeout=30):
        deadline = time.time() + timeout
        while time.time() < deadline:
            st = self.router.plan_status(job_id)
            if st["status"] in ("done", "failed", "cancelled"):
                return st
            time.sleep(0.05)
        self.fail(f"job {job_id} did not finish: {st}")


class RouterTest(RouterBase):
    # ---------------------------------------------------------------- JSON-RPC framing
    def test_jsonrpc_framing(self):
        code, r, headers = R.handle_rpc(self.router, {"jsonrpc": "2.0", "id": 1, "method": "initialize",
                                                      "params": {"protocolVersion": "2025-03-26"}})
        self.assertEqual((code, r["result"]["protocolVersion"]), (200, "2025-03-26"))
        self.assertIn("Mcp-Session-Id", headers)
        self.assertEqual(R.handle_rpc(self.router, {"jsonrpc": "2.0", "method": "notifications/initialized"})[:2],
                         (202, None))
        code, r, _ = R.handle_rpc(self.router, {"jsonrpc": "2.0", "id": 2, "method": "tools/list"})
        self.assertEqual({t["name"] for t in r["result"]["tools"]},
                         {"list_planners", "describe_planner", "recommend_planner", "plan", "plan_status",
                          "cancel_plan", "jobs", "list_plans", "describe_cell", "save_task_program",
                          "task_program_from_products", "check_deployability", "fix_deployability"} | R_FITTER_TOOLS)
        code, r, _ = R.handle_rpc(self.router, {"jsonrpc": "2.0", "id": 3, "method": "tools/call",
                                                "params": {"name": "describe_planner", "arguments": {"name": "nope"}}})
        self.assertTrue(r["result"]["isError"])
        self.assertIn("unknown planner", json.loads(r["result"]["content"][0]["text"])["error"])
        self.assertEqual(R.handle_rpc(self.router, {"jsonrpc": "2.0", "id": 4, "method": "nope"})[0], 404)

    def test_http_server(self):
        httpd = ThreadingHTTPServer(("127.0.0.1", 0), R.make_handler(self.router))
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        try:
            base = f"http://127.0.0.1:{httpd.server_address[1]}"
            with urllib.request.urlopen(base + "/health") as resp:
                self.assertTrue(json.load(resp)["ok"])
            body = json.dumps({"jsonrpc": "2.0", "id": 7, "method": "tools/call",
                               "params": {"name": "list_planners", "arguments": {}}}).encode()
            req = urllib.request.Request(base + "/mcp", data=body, headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req) as resp:
                r = json.load(resp)
            planners = json.loads(r["result"]["content"][0]["text"])["planners"]
            self.assertTrue(next(p for p in planners if p["name"] == "hier_la_fu")["available"])
        finally:
            httpd.shutdown()
            httpd.server_close()

    # ---------------------------------------------------------------- the cell gate
    def test_gate_passes_for_the_training_cell(self):
        ok, why, _ = self.router.gate(self.router.planners["hier_la_fu"], self.CID, self.cell)
        self.assertTrue(ok, why)
        ok, why, warnings = self.router.gate(self.router.planners["ppo_2t"], self.CID, self.cell)
        self.assertTrue(ok, why)  # every planner plans every cell; trained elsewhere is a warning
        self.assertIn("welding_cell_2t", " ".join(warnings))

    def test_gate_fails_when_the_cell_changed(self):
        (self.cell / "scene.yaml").write_text("scene: moved\n")
        ok, why, warnings = self.router.gate(self.router.planners["hier_la_fu"], self.CID, self.cell)
        self.assertTrue(ok)       # no availability gate: the export checks decide
        self.assertIn("scene.yaml (changed)", " ".join(warnings))

    def test_gate_fails_for_another_cell_and_a_missing_checkpoint(self):
        other = self.tmp / "cells/other_cell"
        ok, why, _ = self.router.gate(self.router.planners["hier_q"], "other_cell", other)
        self.assertFalse(ok)
        self.assertIn("no task program", why)
        (other / R.TASKS_DIR).mkdir(parents=True)
        (other / R.TASKS_DIR / "welds.json").write_text("{}")
        ok, why, warnings = self.router.gate(self.router.planners["hier_q"], "other_cell", other)
        self.assertTrue(ok)
        self.assertIn("outside hier_q's training cell", " ".join(warnings))
        self.router.planners["hier_q"]["checkpoint"] = "runs/missing.pt"
        ok, why, _ = self.router.gate(self.router.planners["hier_q"], self.CID, self.cell)
        self.assertFalse(ok)
        self.assertIn("missing", why)

    def test_view_uses_the_factoryos_cells_robots(self):
        view = self.router.build_view(self.CID, self.cell)
        c = view / "cells" / self.CID
        self.assertEqual((c / "robots").resolve(), (self.cell / "robots").resolve())
        self.assertEqual((c / "agentic_planner").resolve(),
                         (self.tmp / "umarell/cells" / self.CID / "agentic_planner").resolve())

    # ---------------------------------------------------------------- routing
    def test_canonical_program_recommends_hier_la_fu(self):
        r = self.router.recommend_planner()
        self.assertEqual(r["recommended"], "hier_la_fu")
        self.assertFalse(r["ambiguous"])
        names = [c["name"] for c in r["candidates"]]
        self.assertEqual(names[:2], ["hier_la_fu", "hier_q"])
        self.assertIn("umarell", names)
        self.assertEqual(self.router.recommend_planner(profile="full")["recommended"], "hier_q")
        self.assertEqual(self.router.recommend_planner(request="Plan the cell.")["recommended"], "hier_la_fu")

    def test_explicit_planner_wins(self):
        r = self.router.recommend_planner(planner="ppo_v3")
        self.assertEqual((r["recommended"], r["ambiguous"]), ("ppo_v3", False))
        r = self.router.recommend_planner(request="please use ppo_pure")
        self.assertEqual(r["recommended"], "ppo_pure")
        r = self.router.recommend_planner(planner="ppo_2t")   # trained on another cell: still the user's choice
        self.assertEqual((r["recommended"], r["ambiguous"]), ("ppo_2t", False))

    def test_a_custom_task_is_planned_by_the_policies_too(self):
        tasks = self.cell / "agentic_planner" / "tasks"
        tasks.mkdir(parents=True)
        (tasks / "subset.json").write_text('{"operations": [{"id": "W02"}]}')
        r = self.router.recommend_planner(task="subset")
        self.assertEqual((r["recommended"], r["canonical_task"]), ("hier_la_fu", False))
        r = self.router.recommend_planner(task="subset", planner="ppo_v4")
        self.assertEqual((r["recommended"], r["ambiguous"]), ("ppo_v4", False))
        job = self.router.plan("ppo_v4", task="subset")
        self.assertEqual(self.router.jobs[job["job_id"]].task_file, tasks / "subset.json")
        view = self.router.build_view(self.CID, self.cell, tasks / "subset.json")
        planner_task = view / "cells" / self.CID / "agentic_planner/tasks/car_door_spot_welding.json"
        self.assertEqual(planner_task.resolve(), (tasks / "subset.json").resolve())
        self.router.cancel_plan(job["job_id"])
        # an unchanged copy of the canonical program is canonical
        shutil.copy(self.tmp / "umarell/cells" / self.CID / "agentic_planner/tasks/car_door_spot_welding.json",
                    tasks / "car_door_spot_welding.json")
        self.assertEqual(self.router.recommend_planner(task="car_door_spot_welding")["recommended"], "hier_la_fu")

    def test_free_text(self):
        r = self.router.recommend_planner(request="avoid the left fixture and weld W03 last")
        self.assertEqual(r["recommended"], "umarell")
        r = self.router.recommend_planner(request="something with the door")
        self.assertTrue(r["ambiguous"])
        self.assertIn("umarell", r["question"])
        with self.assertRaises(R.ToolError):
            self.router.plan("auto", request="something with the door")

    def test_other_cell_goes_to_umarell(self):
        r = self.router.recommend_planner(cell="other_cell")
        self.assertEqual(r["recommended"], "umarell")

    def test_describe_cell_and_save_task_program(self):
        d = self.router.describe_cell()
        self.assertEqual(d["cell_dir"], str(self.cell))
        self.assertIn("operations", d["task_program_format"])
        r = self.router.save_task_program([{"position": [0.1, 0.2, 0.3]}, {"id": "X2", "position": [0, 0, 0.3]}],
                                          name="cube_welds")
        prog = json.loads(Path(r["path"]).read_text())
        self.assertEqual((prog["program"], len(prog["operations"])), ("cube_welds", 2))
        self.assertEqual(prog["operations"][0]["id"], "W01")
        self.assertEqual(prog["operations"][0]["approach_vector"], [0.0, 0.0, -1.0])
        self.assertIn("cube_welds", self.router.describe_cell()["task_programs"])
        with self.assertRaises(R.ToolError):
            self.router.save_task_program([{"position": [0, 0]}])
        with self.assertRaises(R.ToolError):
            self.router.save_task_program([{"position": [0, 0, 0]}], name="../x")

    def test_a_policy_needs_the_cells_weld_ik_table(self):
        self.other_cell(prepared=False)
        (self.tmp / "src" / "data" / "weld_ik_other_cell.json").unlink()
        with self.assertRaises(R.ToolError) as e:
            self.router.plan("hier_la_fu", cell="other_cell")
        self.assertIn("needs the weld IK table data/weld_ik_other_cell.json", str(e.exception.result["error"]))

    def test_plan_any_cell(self):
        # a cell other than the active one: its own gate decides (no task program: not available) ...
        with self.assertRaises(R.ToolError) as e:
            self.router.plan("hier_la_fu", cell="other_cell")
        self.assertIn("not available for cell other_cell", str(e.exception.result["error"]))
        # ... and with one it is planned there: prepared (Prepare Cell for that cell), published into its plans
        other = self.other_cell(prepared=False)
        r = self.router.plan("hier_la_fu", cell="other_cell")
        self.assertIn("not the active cell", r["note"])
        st = self.wait(r["job_id"])
        self.assertEqual(st["status"], "done", st)
        self.assertEqual(self.umarell.prepare_cells, ["other_cell"])
        self.assertTrue((other / R.PREPARED_MARKER).is_file())
        self.assertTrue((other / st["plan_path"]).is_file())
        self.assertFalse((self.cell / R.PLANS_DIR).exists())
        self.assertEqual(self.router.list_plans("other_cell")["plans"][0]["tree_id"], st["tree_id"])

    # ---------------------------------------------------------------- jobs, exit codes, publishing
    def test_command_line(self):
        self.set_spec("ppo_v4", "fake:a,sleep=0.2")
        r = self.router.plan("ppo_v4")
        job = self.router.jobs[r["job_id"]]
        cmd = job.cmd
        self.assertEqual(cmd[:3], [sys.executable, "-m", "tools.btexport"])
        self.assertEqual(cmd[cmd.index("--from") + 1], "fake:a,sleep=0.2")
        self.assertIn("--home-tail", cmd)
        self.assertEqual(cmd[cmd.index("--cell") + 1], self.CID)
        self.assertEqual(cmd[cmd.index("--profile") + 1], "umarell")
        self.assertEqual(cmd[cmd.index("--tree-id") + 1], r["tree_id"])
        for flag in ("--out", "--work", "--report", "--repo"):
            self.assertTrue(cmd[cmd.index(flag) + 1].startswith(str(self.tmp / "work")), flag)
        self.wait(r["job_id"])
        self.assertEqual(self.router.plan("hier_q")["profile"], "full")
        t = self.router.plan("trajgen", profile="full")
        self.assertEqual(self.router.jobs[t["job_id"]].spec,
                         f"plan:trajgen/samples/{self.CID}/expected_full/plan.json")

    def test_done_publishes_only_the_plan(self):
        r = self.router.plan("hier_la_fu")
        st = self.wait(r["job_id"])
        self.assertEqual(st["status"], "done", st)
        self.assertTrue(st["export"]["passed"])
        self.assertEqual(st["export"]["end_s"], 53.76)
        plans = self.cell / R.PLANS_DIR
        self.assertEqual(sorted(p.name for p in plans.iterdir()), [f"{r['tree_id']}.json"])
        plan = json.loads((plans / f"{r['tree_id']}.json").read_text())
        self.assertEqual((plan["planner"], plan["status"], plan["task"]), ("hier_la_fu", "ok", "car_door_spot_welding"))
        self.assertEqual(plan["planner_meta"]["cell_planner_commit"], "abc1234")
        self.assertEqual(plan["planner_meta"]["profile"], "umarell")
        self.assertEqual(st["plan_path"], f"{R.PLANS_DIR}/{r['tree_id']}.json")
        listed = self.router.list_plans()["plans"]
        self.assertEqual((listed[0]["planner"], listed[0]["end_s"]), ("hier_la_fu", 53.76))

    def test_exit_1_publishes_unverified(self):
        self.set_spec("hier_q", "fake:a,exit=1")
        r = self.router.plan("hier_q", profile="umarell")
        st = self.wait(r["job_id"])
        self.assertEqual(st["status"], "done")
        self.assertFalse(st["export"]["passed"])
        self.assertIn("limits", st["export"]["failed_checks"])
        self.assertIn("unverified", st["hint"])
        plan = json.loads((self.cell / st["plan_path"]).read_text())
        self.assertEqual(plan["status"], "unverified")

    def test_exit_1_without_report_is_a_crash(self):
        self.set_spec("hier_la_fu", "fake:a,crash")
        st = self.wait(self.router.plan("hier_la_fu")["job_id"])
        self.assertEqual(st["status"], "failed", st)
        self.assertEqual(st["export"]["exit_code"], 1)
        self.assertIn("crashed", st["error"])
        self.assertIn("nothing was published", st["hint"])
        self.assertFalse((self.cell / R.PLANS_DIR).exists() and any((self.cell / R.PLANS_DIR).iterdir()))

    def test_exit_2_and_timeout_publish_nothing(self):
        self.set_spec("ppo_pure", "fake:a,exit=2")
        st = self.wait(self.router.plan("ppo_pure")["job_id"])
        self.assertEqual(st["status"], "failed")
        self.assertIn("exit 2", st["error"])
        self.set_spec("ppo_v3", "fake:a,sleep=30")
        self.router.planners["ppo_v3"]["timeout_s"] = 0.5
        st = self.wait(self.router.plan("ppo_v3")["job_id"], timeout=20)
        self.assertEqual(st["status"], "failed")
        self.assertIn("timed out", st["error"])
        self.assertFalse((self.cell / R.PLANS_DIR).exists() and any((self.cell / R.PLANS_DIR).iterdir()))

    def test_one_worker_queues_and_cancel(self):
        self.router = self.make_router(workers=1)
        self.set_spec("ppo_v3", "fake:a,sleep=30")
        a = self.router.plan("ppo_v3")
        b = self.router.plan("hier_la_fu")
        self.assertEqual(b["status"], "queued")
        self.assertEqual(b["running"] + b["queued_behind"], [a["job_id"]])
        self.assertEqual(b["workers"]["planner"], 1)
        self.assertNotEqual(a["tree_id"], b["tree_id"])
        self.assertTrue(self.router.cancel_plan(b["job_id"])["cancelled"])
        deadline = time.time() + 10
        while self.router.plan_status(a["job_id"])["status"] != "planning" and time.time() < deadline:
            time.sleep(0.05)
        self.assertTrue(self.router.cancel_plan(a["job_id"])["cancelled"])
        self.assertEqual(self.wait(a["job_id"])["status"], "cancelled")
        self.assertEqual(self.router.plan_status(b["job_id"])["status"], "cancelled")

    def test_tree_id_collision_bump(self):
        self.set_spec("hier_la_fu", "fake:a,sleep=0.3")
        r = self.router.plan("hier_la_fu")
        plans = self.cell / R.PLANS_DIR
        plans.mkdir(parents=True, exist_ok=True)
        (plans / f"{r['tree_id']}.json").write_text('{"behaviour_tree": {}}')   # umarell, the same second
        st = self.wait(r["job_id"])
        self.assertNotEqual(st["tree_id"], r["tree_id"])
        plan = json.loads((self.cell / st["plan_path"]).read_text())
        self.assertEqual(plan["tree_id"], st["tree_id"])
        self.assertEqual(plan["behaviour_tree"]["tree_id"], st["tree_id"])
        # and the next reservation skips both
        self.assertNotIn(self.router._reserve_tree_id(self.cell, self.CID), (r["tree_id"], st["tree_id"]))

    def test_unavailable_planner_is_refused(self):
        self.router.planners["ppo_2t"]["checkpoint"] = "runs/missing.pt"   # can't run here at all
        with self.assertRaises(R.ToolError) as e:
            self.router.plan("ppo_2t")
        self.assertIn("not available", e.exception.result["error"])

    # ---------------------------------------------------------------- the umarell route
    def test_umarell_forward(self):
        r = self.router.plan("umarell")
        self.assertEqual((r["planner"], r["job_id"], r["status"]), ("umarell", f"bt_1700000000_{self.CID}", "planning"))
        name, args = self.last_plan_call()
        self.assertEqual((name, args["cell_id"], args["operations"]), ("plan_trajectories", self.CID, [{"id": "W01"}]))
        st = self.router.plan_status(r["job_id"])
        self.assertEqual((st["status"], st["plan_path"]), ("done", f"{R.PLANS_DIR}/{r['job_id']}.json"))
        self.assertEqual(self.umarell.calls[-1], ("get_plan_status", {"tree_id": r["job_id"]}))
        self.assertEqual(self.router.jobs_list()["jobs"][0]["status"], "done")

    def test_umarell_prepares_the_cell_first(self):
        # an unprepared active cell: Prepare Cell runs once, then umarell plans
        self.assertFalse((self.cell / R.PREPARED_MARKER).exists())
        self.router.plan("umarell")
        self.assertEqual(self.umarell.prepare_calls, 1)
        self.assertTrue((self.cell / R.PREPARED_MARKER).is_file())
        self.router.plan("umarell")
        self.assertEqual(self.umarell.prepare_calls, 1)

    def test_umarell_prepare_failure_is_an_error(self):
        prepare, self.umarell.prepare = self.umarell.prepare, None
        n = len(self.umarell.calls)
        with self.assertRaises(R.ToolError) as e:
            self.router.plan("umarell")
        self.assertIn("Prepare Cell failed", e.exception.result["error"])
        self.assertFalse([c for c in self.umarell.calls[n:] if c[0] == "plan_trajectories"])
        # another cell: Prepare Cell for that cell, then umarell plans it
        self.umarell.prepare = prepare
        self.other_cell(task="car_door_spot_welding", prepared=False)
        r = self.router.plan("umarell", cell="other_cell")
        self.assertEqual(self.umarell.prepare_cells[-1], "other_cell")
        self.assertEqual(self.last_plan_call()[1]["cell_id"], "other_cell")
        self.assertEqual(r["cell"], "other_cell")

    def test_an_old_backend_prepares_the_active_cell_only(self):
        self.umarell.old_backend = True
        self.other_cell(task="car_door_spot_welding", prepared=False)
        n = len(self.umarell.calls)
        with self.assertRaises(R.ToolError) as e:
            self.router.plan("umarell", cell="other_cell")
        self.assertIn("not prepared", e.exception.result["error"])
        self.assertFalse([c for c in self.umarell.calls[n:] if c[0] == "plan_trajectories"])
        self.router.plan("umarell")                       # the active cell: as before
        self.assertTrue((self.cell / R.PREPARED_MARKER).is_file())

    def test_umarell_free_text_is_not_dropped(self):
        n = len(self.umarell.calls)
        with self.assertRaises(R.ToolError) as e:
            self.router.plan("umarell", request="avoid W03")
        self.assertIn("free text", e.exception.result["error"])
        self.assertIn("options.operations", e.exception.result["hint"])
        self.assertFalse([c for c in self.umarell.calls[n:] if c[0] == "plan_trajectories"])
        # encoded as operations / constraints, or marked applied: forwarded
        self.router.plan("umarell", request="avoid W03", options={"operations": [{"id": "W01"}]})
        self.assertEqual(self.last_plan_call()[1]["operations"], [{"id": "W01"}])
        self.router.plan("umarell", request="at most 2 robots at once",
                         options={"constraints": {"max_parallel": 2}})
        self.assertEqual(self.last_plan_call()[1]["constraints"], {"max_parallel": 2})
        self.router.plan("umarell", request="avoid W03", options={"request_applied": True})
        # a plain request with the planner's name is no wish
        for text in ("plan the cell with umarell", "Plan Cell", "use umarell please"):
            self.assertEqual(self.router.plan("umarell", request=text)["planner"], "umarell", text)

    def test_umarell_down(self):
        self.router.s.umarell_url = "http://127.0.0.1:9/mcp"
        R.UMARELL_START_WAIT_S, old = 0.5, R.UMARELL_START_WAIT_S
        try:
            with self.assertRaises(R.ToolError) as e:
                self.router.plan("umarell")
            self.assertEqual(e.exception.result["hint"], "Start simulation in Agentic Planning")
        finally:
            R.UMARELL_START_WAIT_S = old


class ParallelTest(RouterBase):
    """Jobs in parallel: other cells, the same cell, the worker caps, umarell's slots, the jobs tool."""

    def watch(self, job_ids, timeout=30):
        """Wait for the jobs; returns their final status and the most that were planning at once (by pool)."""
        most, deadline = {"planner": 0, "umarell": 0}, time.time() + timeout
        while time.time() < deadline:
            with self.router.lock:
                live = [j for j in self.router.jobs.values() if j.status == "planning"]
                for pool in most:
                    most[pool] = max(most[pool], sum(1 for j in live if self.router._pool(j) == pool))
            done = [self.router.plan_status(j) for j in job_ids]
            if all(st["status"] in R.Router.TERMINAL for st in done):
                return done, most
            time.sleep(0.02)
        self.fail(f"jobs did not finish: {[self.router.plan_status(j)['status'] for j in job_ids]}")

    def test_two_cells_run_at_the_same_time(self):
        self.router = self.make_router(workers=2)
        other = self.other_cell()
        self.set_spec("hier_la_fu", "fake:a,sleep=1")
        self.set_spec("hier_q", "fake:a,sleep=1")
        t0 = time.time()
        a = self.router.plan("hier_la_fu")
        b = self.router.plan("hier_q", cell="other_cell")
        self.assertEqual((a["status"], b["status"]), ("planning", "planning"))
        self.assertEqual(b["queued_behind"], [])
        (sa, sb), most = self.watch([a["job_id"], b["job_id"]])
        self.assertEqual((sa["status"], sb["status"]), ("done", "done"), (sa, sb))
        self.assertEqual(most["planner"], 2)
        self.assertLess(time.time() - t0, 1.9 + 1.5)      # both slept 1 s, side by side (+ start-up)
        self.assertTrue((self.cell / sa["plan_path"]).is_file())
        self.assertTrue((other / sb["plan_path"]).is_file())
        self.assertEqual(json.loads((other / sb["plan_path"]).read_text())["cell_id"], "other_cell")

    def test_same_cell_jobs_dont_clash(self):
        self.router = self.make_router(workers=3)
        for p in ("hier_la_fu", "ppo_v3", "ppo_pure"):
            self.set_spec(p, "fake:a,sleep=0.6")
        self.set_spec("ppo_pure", "fake:a,sleep=1.5")
        rs = [self.router.plan(p) for p in ("hier_la_fu", "ppo_v3", "ppo_pure")]
        self.assertEqual(len({r["tree_id"] for r in rs}), 3)
        # umarell saved a plan with the first job's id meanwhile: it bumps, but never onto a live job's id
        plans = self.cell / R.PLANS_DIR
        plans.mkdir(parents=True, exist_ok=True)
        (plans / f"{rs[0]['tree_id']}.json").write_text('{"behaviour_tree": {}}')
        done, most = self.watch([r["job_id"] for r in rs])
        self.assertEqual([st["status"] for st in done], ["done"] * 3, done)
        self.assertEqual(most["planner"], 3)
        ids = [st["tree_id"] for st in done]
        self.assertEqual(len(set(ids) | {rs[0]["tree_id"]}), 4, ids)
        for st, r in zip(done, rs):
            plan = json.loads((self.cell / st["plan_path"]).read_text())
            self.assertEqual((plan["tree_id"], plan["planner"]), (st["tree_id"], r["planner"]))
        # every job had its own view (the fake btexport checks it after its sleep), all gone now
        views = {self.router.jobs[r["job_id"]].view for r in rs}
        self.assertEqual(len(views), 3)
        # a worker removes its job's view just after the job turns done
        deadline = time.time() + 5
        while any(v.exists() for v in views) and time.time() < deadline:
            time.sleep(0.02)
        self.assertFalse(any(v.exists() for v in views))
        self.assertEqual(self.router.reserved, set())

    def test_the_worker_cap(self):
        self.router = self.make_router(workers=2)
        for p in ("hier_la_fu", "hier_q", "ppo_v3", "ppo_v4"):
            self.set_spec(p, "fake:a,sleep=0.7")
        rs = [self.router.plan(p) for p in ("hier_la_fu", "hier_q", "ppo_v3", "ppo_v4")]
        self.assertEqual([r["status"] for r in rs], ["planning", "planning", "queued", "queued"])
        self.assertEqual(rs[3]["queued_behind"], [rs[2]["job_id"]])
        self.assertEqual(sorted(rs[3]["running"]), sorted([rs[0]["job_id"], rs[1]["job_id"]]))
        self.assertGreater(rs[3]["starts_in_s"], rs[2]["starts_in_s"] - 1e-9)
        self.assertGreater(rs[3]["eta_s"], rs[3]["starts_in_s"])
        listed = self.router.jobs_list()
        self.assertEqual((listed["planning"] + listed["queued"], listed["workers"]["planner"]), (4, 2))
        self.assertEqual({j["cell"] for j in listed["jobs"]}, {self.CID})
        self.assertEqual({j["planner"] for j in listed["jobs"]}, {"hier_la_fu", "hier_q", "ppo_v3", "ppo_v4"})
        for j in listed["jobs"]:
            self.assertIn(j["status"], ("planning", "queued"))
            self.assertIn("elapsed_s", j)
        result, is_error = R.call_tool(self.router, "jobs", {"active_only": True})
        self.assertFalse(is_error, result)
        done, most = self.watch([r["job_id"] for r in rs])
        self.assertEqual([st["status"] for st in done], ["done"] * 4, done)
        self.assertEqual(most["planner"], 2)
        self.assertEqual(R.call_tool(self.router, "jobs", {"active_only": True})[0]["jobs"], [])

    def test_workers_from_the_environment(self):
        old = os.environ.get("FACTORYOS_PLANNER_WORKERS")
        os.environ["FACTORYOS_PLANNER_WORKERS"] = "3"
        try:
            s = R.Settings(cell_planner=self.tmp / "src", python=Path(sys.executable), umarell=self.tmp / "umarell",
                           config=self.config, work=self.tmp / "work", planners_file=self.planners)
            r = R.Router(s)
            self.assertEqual((r.workers, r.umarell_workers), (3, 2))
        finally:
            if old is None:
                os.environ.pop("FACTORYOS_PLANNER_WORKERS")
            else:
                os.environ["FACTORYOS_PLANNER_WORKERS"] = old
        self.assertGreaterEqual(R.env_int("NO_SUCH_VARIABLE_X", max(2, R.CPUS)), 2)

    def test_umarell_slots(self):
        R.UMARELL_POLL_S, old = 0.05, R.UMARELL_POLL_S
        self.addCleanup(setattr, R, "UMARELL_POLL_S", old)
        self.router = self.make_router(umarell_workers=1)
        self.umarell.status = "planning"
        self.other_cell(task="car_door_spot_welding")
        a = self.router.plan("umarell")
        b = self.router.plan("umarell", cell="other_cell")
        self.assertEqual((a["status"], b["status"]), ("planning", "queued"))
        self.assertTrue(a["job_id"].startswith("bt_"))
        self.assertEqual((b["running"], b["tree_id"]), ([a["job_id"]], None))
        self.assertEqual(b["workers"]["umarell"], 1)
        self.assertGreater(b["starts_in_s"], 0)
        time.sleep(0.3)
        self.assertEqual(len([c for c in self.umarell.calls if c[0] == "plan_trajectories"]), 1)
        self.assertEqual(self.router.plan_status(b["job_id"], wait_s=0)["status"], "queued")
        # a policy job meanwhile: other workers, not umarell's slot
        self.set_spec("hier_la_fu", "fake:a")
        self.assertEqual(self.wait(self.router.plan("hier_la_fu")["job_id"])["status"], "done")
        self.umarell.status = "done"         # the first frees its slot: the second goes to umarell
        sb = self.wait(b["job_id"])
        self.assertEqual(sb["status"], "done", sb)
        self.assertEqual(sb["cell"], "other_cell")
        self.assertEqual(sb["plan_path"], f"{R.PLANS_DIR}/{sb['tree_id']}.json")
        self.assertEqual(self.last_plan_call()[1]["cell_id"], "other_cell")
        self.assertEqual(self.router.plan_status(sb["tree_id"])["job_id"], b["job_id"])   # by umarell's id too
        self.assertEqual(self.router.plan_status(a["job_id"])["status"], "done")

    def test_umarell_cancel_queued(self):
        self.router = self.make_router(umarell_workers=1)
        self.umarell.status = "planning"
        a = self.router.plan("umarell")
        b = self.router.plan("umarell")
        self.assertTrue(self.router.cancel_plan(b["job_id"])["cancelled"])
        self.assertEqual(self.router.plan_status(b["job_id"])["status"], "cancelled")
        r = self.router.cancel_plan(a["job_id"])
        self.assertTrue(r["cancelled"])
        self.assertIn(("cancel_planning", {"tree_id": a["job_id"], "reason": "cancelled by the user"}),
                      self.umarell.calls)


class HelpersTest(unittest.TestCase):
    def test_planner_cell_id(self):
        self.assertEqual(R.planner_cell_id("welding_cell_2_small"), "welding_cell_2_small")
        self.assertEqual(R.planner_cell_id("2-cell"), "cell_2_cell")

    def test_cell_root(self):
        self.assertEqual(R.cell_root("/c/x/cellrunner"), Path("/c/x"))
        self.assertEqual(R.cell_root("/c/x"), Path("/c/x"))

    def test_real_catalogue(self):
        cat = json.loads((ROUTER_DIR / "planners.json").read_text())
        names = [p["name"] for p in cat["planners"]]
        self.assertEqual(names[:2], ["hier_la_fu", "hier_q"])
        for p in cat["planners"]:
            if p["kind"] != "llm":
                for c in p["cells"]:
                    self.assertIn(c, cat["training_cells"], p["name"])


# jam2 (Oct 2026): the product bug_cube (a 2 m box, spot_weld_1: 4 welds on the top face) placed as a fixture; its
# obstacles.yaml is the voxel shell the converter writes (6 boxes, none is_workpiece).
BUG_CUBE = {"name": "bug_cube", "description": "", "geometry": {"kind": "box", "size": [2.0, 2.0, 2.0]},
            "tasks": [{"name": "spot_weld_1", "process": "spotWeld", "tool": "",
                       "params": {"approachOffset": 0.05, "current": 9.0, "force": 3.0, "weldTime": 0.3},
                       "operations": [{"id": f"W0{i + 1}", "group": "structural",
                                       "target": {"kind": "point", "position": [x, y, 2.0], "normal": [0.0, 0.0, 1.0]}}
                                      for i, (x, y) in enumerate([(-0.5, -0.5), (0.5, -0.5), (-0.5, 0.5), (0.5, 0.5)])]}]}

JAM2_SCENE = """name: jam2
robots:
- robot_id: kr240r2900__sticktool
  dressed_urdf: robots/kr240r2900__sticktool/kr240r2900__sticktool.urdf
  position:
    x: 2.5
    y: 0.0
    z: 0.0
  orientation:
    a: 180.0
    b: 0.0
    c: 0.0
  home_pose_deg:
  - 45.0
  - -90.0
  pedestal:
    height_mm: 1000.0
    diameter_mm: 1500.0
fixtures:
- id: bug_cube
  urdf: fixtures/bug_cube/bug_cube.urdf
  position:
    x: {x}
    y: {y}
    z: {z}
  orientation:
    a: {a}
    b: 0.0
    c: 0.0
"""

JAM2_OBSTACLES = """# Static collision obstacles for cell 'jam2'
# is_workpiece: set true on the box(es) covering the surface being welded
obstacles:

  # Cell floor
  - id: floor
    type: plane
    description: "Cell floor"
    normal: [0.0, 0.0, 1.0]
    offset: -0.05
    collision_margin: 0.05

  # bug_cube: voxel-occupancy shell
  - id: bug_cube_v0
    type: box
    description: "bug_cube voxel shell 0"
    center: [0.025, 0.025, 0.025]
    half_extents: [1.025, 1.025, 0.025]
    collision_margin: 0.05
  - id: bug_cube_v1
    type: box
    description: "bug_cube voxel shell 1"
    center: [0.025, -0.975, 1.05]
    half_extents: [1.025, 0.025, 1]
    collision_margin: 0.05
  - id: bug_cube_v3
    type: box
    description: "bug_cube voxel shell 3"
    center: [0.05, 0.05, 2.025]
    half_extents: [1, 1, 0.025]
    collision_margin: 0.05

  - id: pedestal_robot1
    type: box
    description: "robot1 pedestal"
    center: [2.5, 0.0, 0.4]
    half_extents: [0.75, 0.75, 0.4]
    collision_margin: 0.05

default_collision_margin: 0.05
"""


class ProductsTest(RouterBase):
    """describe_cell's products, task_program_from_products and save_task_program's checks, on a jam2-like cell."""

    def place(self, x=0.0, y=0.0, z=0.0, a=0.0, product=BUG_CUBE, obstacles=JAM2_OBSTACLES):
        (self.cell / "scene.yaml").write_text(JAM2_SCENE.format(x=x, y=y, z=z, a=a))
        d = self.cell / "fixtures" / "bug_cube"
        d.mkdir(parents=True, exist_ok=True)
        (d / "bug_cube.urdf").write_text("<robot/>")
        (d / "product.json").write_text(json.dumps(product))
        o = self.cell / "agentic_planner" / "config" / "obstacles.yaml"
        o.parent.mkdir(parents=True, exist_ok=True)
        o.write_text(obstacles)
        return o

    def test_jam2_top_welds_in_the_world(self):
        self.place()
        d = self.router.describe_cell()
        self.assertEqual(d["product_welds"], 4)
        self.assertIn("task_program_from_products", d["next"])
        (p,) = d["products"]
        self.assertEqual((p["fixture_id"], p["product"]), ("bug_cube", "bug_cube"))
        self.assertEqual([w["position"] for w in p["welds"]],
                         [[-0.5, -0.5, 2.0], [0.5, -0.5, 2.0], [-0.5, 0.5, 2.0], [0.5, 0.5, 2.0]])
        for w in p["welds"]:
            self.assertEqual(w["approach_vector"], [0.0, 0.0, -1.0])
            self.assertEqual(w["orientation"], {"x": 0.0, "y": -0.7071, "z": 0.0, "w": 0.7071})

    def test_task_program_from_products(self):
        obstacles = self.place()
        result, is_error = R.call_tool(self.router, "task_program_from_products", {})
        self.assertFalse(is_error, result)
        self.assertEqual(result["task"], "bug_cube_spot_welding")
        self.assertEqual(result["workpiece_boxes"], ["bug_cube_v3"])
        prog = json.loads(Path(result["path"]).read_text())
        self.assertEqual([o["id"] for o in prog["operations"]], ["W01", "W02", "W03", "W04"])
        for o in prog["operations"]:
            self.assertEqual(o["position"][2], 2.0)
            self.assertEqual(o["approach_vector"], [0.0, 0.0, -1.0])
            self.assertEqual(o["orientation"], R.TOP_DOWN)
            self.assertEqual(o["weld_orientation"], R.TOP_DOWN)
            self.assertEqual(o["sequence_group"], "structural")
        boxes = {b["id"]: b["is_workpiece"] for b in R.obstacle_boxes(obstacles.read_text())}
        self.assertEqual(boxes, {"bug_cube_v0": False, "bug_cube_v1": False, "bug_cube_v3": True,
                                 "pedestal_robot1": False})
        text = obstacles.read_text()
        self.assertIn("# Static collision obstacles", text)                       # comments kept
        self.assertEqual(text.count("is_workpiece: true"), 1)
        # again: the box is marked already, nothing changes
        self.router.task_program_from_products(name="again")
        self.assertEqual(obstacles.read_text(), text)

    def test_workpiece_marking_knows_the_fine_shell(self):
        # The same cases as product_tasks.rs marks_the_welded_fixtures_fine_shell_within_the_refine_radius.
        def b(i, c, h):
            return {"id": i, "center": c, "half_extents": [h] * 3, "is_workpiece": False}
        boxes = [b("door_f12", [0.0, 0.0, 0.0], 0.01), b("door_f13", [0.2, 0.0, 0.0], 0.01),
                 b("door_f14", [0.3, 0.0, 0.0], 0.01), b("door_v0007", [0.0, 0.2, 0.0], 0.025),
                 b("door_v0008", [0.0, 0.1, 0.0], 0.025), b("clamp_f1", [0.0, -0.05, 0.0], 0.01)]
        self.assertEqual(R.workpiece_ids(boxes, [[0.0, 0.0, 0.01]]), {"door_f12", "door_f13", "door_v0008"})
        self.assertEqual(R.shell_of("body_weld_1_f233"), "body_weld_1")
        self.assertEqual(R.shell_of("body_weld_1_v0233"), "body_weld_1")
        for not_shell in ("floor", "door_f", "_f1", "x_fv1"):
            self.assertIsNone(R.shell_of(not_shell), not_shell)
        self.assertEqual((R.shell_near_m("a_f1"), R.shell_near_m("a_v1")), (R.REFINE_RADIUS_M, R.WORKPIECE_NEAR_M))
        text = "obstacles:\n  - id: door_f12\n    type: box\n    center: [0, 0, 0]\n    half_extents: [0.01, 0.01, 0.01]\n"
        self.assertIn("    is_workpiece: true", R.mark_workpieces(text, {"door_f12"}))

    def test_a_turned_and_moved_fixture(self):
        side = json.loads(json.dumps(BUG_CUBE))
        side["tasks"][0]["operations"].append(
            {"id": "S01", "target": {"kind": "point", "position": [1.0, 0.0, 1.0], "normal": [1.0, 0.0, 0.0]}})
        side["tasks"][0]["operations"].append(
            {"id": "P01", "target": {"kind": "path", "points": [[0, 0, 2], [0.5, 0, 2]], "normal": [0, 0, 1]}})
        self.place(x=1.0, y=2.0, z=0.5, a=90.0, product=side, obstacles="obstacles: []\n")
        (p,) = self.router.describe_cell()["products"]
        w = {w["id"]: w for w in p["welds"]}
        # Rz(90): (x, y) -> (-y, x)
        self.assertEqual(w["W02"]["position"], [1.5, 2.5, 2.5])
        self.assertEqual(w["W02"]["approach_vector"], [0.0, 0.0, -1.0])
        self.assertEqual(w["S01"]["position"], [1.0, 3.0, 1.5])
        self.assertEqual(w["S01"]["approach_vector"], [0.0, -1.0, 0.0])
        self.assertLess(R.angle_deg(R.tool_axis(w["S01"]["orientation"]), [0, -1, 0]), 0.1)
        self.assertEqual(p["skipped_paths"], ["spot_weld_1/P01 (path)"])
        r = self.router.task_program_from_products(name="turned")
        self.assertEqual(r["operations"], 5)

    def test_the_backends_numbers(self):
        # crates/factoryos-backend services::product_tasks: the same fixture (1, 2, 0) turned 90°, the same welds
        side = json.loads(json.dumps(BUG_CUBE))
        side["tasks"][0]["operations"].append(
            {"id": "S01", "group": "side", "params": {"priority": 2},
             "target": {"kind": "point", "position": [1.0, 0.0, 1.0], "normal": [1.0, 0.0, 0.0]}})
        self.place(x=1.0, y=2.0, z=0.0, a=90.0, product=side, obstacles="obstacles: []\n")
        (p,) = self.router.describe_cell()["products"]
        w = {w["id"]: w for w in p["welds"]}
        self.assertEqual(w["W02"]["position"], [1.5, 2.5, 2.0])
        self.assertEqual(w["S01"]["position"], [1.0, 3.0, 1.0])
        self.assertEqual(w["S01"]["approach_vector"], [0.0, -1.0, 0.0])
        self.assertEqual(w["S01"]["orientation"], {"x": -0.5, "y": -0.5, "z": 0.5, "w": 0.5})
        self.assertEqual((w["S01"]["group"], w["S01"]["priority"]), ("side", 2))
        self.assertEqual(R.orientation_for([1, 0, 0]), {"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0})
        self.assertEqual(R.orientation_for([0, 0, 1]), {"x": -0.7071, "y": 0.0, "z": 0.7071, "w": 0.0})

    def test_backend_derived_task_programs_are_preferred(self):
        obstacles = self.place()
        tasks = self.cell / R.TASKS_DIR
        tasks.mkdir(parents=True, exist_ok=True)
        ops = [{"id": f"W0{i + 1}", "type": "weld", "position": [x, y, 2.0], "approach_vector": [0.0, 0.0, -1.0],
                "orientation": R.TOP_DOWN, "weld_orientation": R.TOP_DOWN, "sequence_group": "structural",
                "priority": 1} for i, (x, y) in enumerate([(-0.5, -0.5), (0.5, -0.5), (-0.5, 0.5), (0.5, 0.5)])]
        derived = {"program": "bug_cube__spot_weld_1", "cell_id": "jam2", "operations": ops,
                   "source": {"product": "bug_cube", "task": "spot_weld_1", "fixture": "bug_cube",
                              "product_sha256": "0" * 64, "program_sha256": "1" * 64}}
        (tasks / "bug_cube__spot_weld_1.json").write_text(json.dumps(derived))
        (tasks / "mine.json").write_text(json.dumps({"program": "mine", "operations": ops[:1]}))
        d = self.router.describe_cell()
        self.assertIn("Product & Task", d["next"])
        self.assertIn("bug_cube__spot_weld_1", d["next"])
        self.assertEqual([(t["file"], t["from_product"]) for t in d["task_sources"]],
                         [("bug_cube__spot_weld_1", True), ("mine", False)])
        before = sorted(f.name for f in tasks.iterdir())
        result, is_error = R.call_tool(self.router, "task_program_from_products", {})
        self.assertFalse(is_error, result)
        self.assertEqual((result["source"], result["task"], result["operations"]), ("backend", "bug_cube__spot_weld_1", 4))
        self.assertEqual(result["tasks"][0]["product"], "bug_cube")
        self.assertEqual(sorted(f.name for f in tasks.iterdir()), before, "nothing written")
        self.assertEqual(result["workpiece_boxes"], ["bug_cube_v3"])
        self.assertEqual(obstacles.read_text().count("is_workpiece: true"), 1)
        # an explicit name still writes the combined program
        self.assertEqual(self.router.task_program_from_products(name="combined")["task"], "combined")
        self.assertTrue((tasks / "combined.json").is_file())

    def test_no_product_is_an_error(self):
        (self.cell / "scene.yaml").write_text("name: x\nfixtures: []\n")
        self.assertEqual(self.router.describe_cell()["product_welds"], 0)
        with self.assertRaises(R.ToolError) as e:
            self.router.task_program_from_products()
        self.assertIn("tell the user", e.exception.result["hint"])

    def test_save_task_program_refuses_contradicting_orientations(self):
        self.place()
        # jam2's agent: a side-wall weld approached along +y with the top-down orientation
        with self.assertRaises(R.ToolError) as e:
            self.router.save_task_program([{"id": "W05", "position": [0.0, -1.0, 1.0], "approach_vector": [0, 1, 0],
                                            "orientation": R.TOP_DOWN}], name="bad")
        self.assertIn("off approach_vector", e.exception.result["error"])
        self.assertIn("derived from approach_vector", e.exception.result["hint"])
        # within 10°: accepted; no orientation: derived from approach_vector
        tilted = R.orientation_for([0.1, 0.0, -1.0])
        self.router.save_task_program([{"position": [0.0, 0.0, 2.0], "approach_vector": [0, 0, -1],
                                        "orientation": tilted}], name="ok")
        r = self.router.save_task_program([{"position": [0.0, -1.05, 1.0], "approach_vector": [0, 1, 0]}],
                                          name="derived")
        op = json.loads(Path(r["path"]).read_text())["operations"][0]
        self.assertLess(R.angle_deg(R.tool_axis(op["orientation"]), [0, 1, 0]), 0.1)

    def test_save_task_program_refuses_a_weld_inside_an_obstacle(self):
        self.place()
        with self.assertRaises(R.ToolError) as e:   # inside the side wall bug_cube_v1
            self.router.save_task_program([{"position": [0.0, -0.975, 1.0], "approach_vector": [0, 1, 0]}])
        self.assertIn("bug_cube_v1", e.exception.result["error"])
        with self.assertRaises(R.ToolError):        # inside the robot's pedestal
            self.router.save_task_program([{"position": [2.5, 0.0, 0.4]}])
        # on the surface is fine, and inside a workpiece box too
        self.router.save_task_program([{"position": [0.0, 0.0, 2.0]}])
        self.router.task_program_from_products()
        self.router.save_task_program([{"position": [0.0, 0.0, 2.02]}])


class GeometryTest(unittest.TestCase):
    def test_orientation_convention(self):
        self.assertEqual(R.orientation_for([0, 0, -1]), R.TOP_DOWN)
        self.assertEqual([round(v, 4) for v in R.tool_axis(R.TOP_DOWN)], [0.0, 0.0, -1.0])
        for a in ([1, 0, 0], [0, -1, 0], [0, 0, 1], [0.3, -0.5, -0.2], [-1, 1, 0.5]):
            self.assertLess(R.angle_deg(R.tool_axis(R.orientation_for(a)), a), 0.05, a)

    def test_euler_order(self):
        # Rz(a)·Ry(b)·Rx(c): x turned by c=90 about x, then b=90 about y: z -> ... check one column
        r = R.euler_zyx_deg(90, 0, 0)
        self.assertEqual([round(v, 6) + 0.0 for v in R._apply(r, [1, 0, 0])], [0.0, 1.0, 0.0])
        r = R.euler_zyx_deg(90, 90, 0)   # Rz(90)·Ry(90): x -> -z (Ry) -> -z (Rz)
        self.assertEqual([round(v, 6) + 0.0 for v in R._apply(r, [1, 0, 0])], [0.0, 0.0, -1.0])
        r = R.euler_zyx_deg(90, 0, 90)   # Rz(90)·Rx(90): y -> z (Rx) -> z (Rz)
        self.assertEqual([round(v, 6) + 0.0 for v in R._apply(r, [0, 1, 0])], [0.0, 0.0, 1.0])

    def test_yaml_subset(self):
        doc = R.parse_yaml(JAM2_SCENE.format(x=0.0, y=0.0, z=0.0, a=0.0))
        self.assertEqual(doc["robots"][0]["home_pose_deg"], [45.0, -90.0])
        self.assertEqual(doc["fixtures"][0]["urdf"], "fixtures/bug_cube/bug_cube.urdf")
        self.assertEqual(len(R.obstacle_boxes(JAM2_OBSTACLES)), 4)


class U3Test(RouterBase):
    """umarell_u3: the router runs umarell_u3.py (a fake U3 in the cell planner, this python as umarell's venv)."""

    def setUp(self):
        super().setUp()
        src = self.tmp / "src"
        u3 = src / "umarell_fast" / "u3"
        u3.mkdir(parents=True)
        (src / "umarell_fast" / "__init__.py").write_text("")
        (src / "umarell_fast" / "apply.py").write_text("")
        (u3 / "__init__.py").write_text("")
        (u3 / "__main__.py").write_text(FAKE_U3)
        (src / "docs" / "upstream" / "patches").mkdir(parents=True)
        (src / "docs" / "upstream" / "patches" / "01_x.diff").write_text("")
        self.router.s.umarell_python = Path(sys.executable)
        self.root = self.tmp / "work" / "umarell_u3"
        # The gate wants patch and rsync (umarell_fast/apply.py runs them; the fake U3 does not): the CI build image
        # need not have them.
        self.which = {"patch": "/usr/bin/patch", "rsync": "/usr/bin/rsync"}
        real_which = R.shutil.which
        R.shutil.which = lambda cmd, *a, **k: self.which.get(cmd) if cmd in ("patch", "rsync") else real_which(cmd, *a, **k)
        self.addCleanup(setattr, R.shutil, "which", real_which)

    def test_u3_is_the_default_outside_the_policies_training(self):
        r = self.router.recommend_planner()                 # the canonical program in the training cell
        self.assertEqual(r["recommended"], "hier_la_fu")
        self.assertIn("umarell_u3", [c["name"] for c in r["candidates"]])
        (self.cell / "scene.yaml").write_text("scene: moved\n")
        r = self.router.recommend_planner()
        self.assertEqual((r["recommended"], r["ambiguous"]), ("umarell_u3", False))
        r = self.router.recommend_planner(request="avoid the left fixture and weld W03 last")
        self.assertEqual(r["recommended"], "umarell")       # free-text constraints: the LLM planner
        # a cell without a task program: umarell (the agent derives one); with one: U3
        self.assertEqual(self.router.recommend_planner(cell="other_cell")["recommended"], "umarell")
        other = self.tmp / "cells/other_cell"
        (other / R.TASKS_DIR).mkdir(parents=True)
        (other / R.TASKS_DIR / "welds.json").write_text('{"operations": [{"id": "W01"}]}')
        r = self.router.recommend_planner(cell="other_cell")
        self.assertEqual(r["recommended"], "umarell_u3")

    def test_u3_gate(self):
        ok, why, warnings = self.router.gate(self.router.planners["umarell_u3"], self.CID, self.cell)
        self.assertTrue(ok, why)
        self.assertEqual(warnings, [])
        self.which.pop("rsync")
        ok, why, _ = self.router.gate(self.router.planners["umarell_u3"], self.CID, self.cell)
        self.assertFalse(ok)
        self.assertIn("patch and rsync", why)
        self.which["rsync"] = "/usr/bin/rsync"
        shutil.rmtree(self.tmp / "src" / "umarell_fast")
        ok, why, _ = self.router.gate(self.router.planners["umarell_u3"], self.CID, self.cell)
        self.assertFalse(ok)
        self.assertIn("no umarell_fast/u3", why)
        self.assertNotEqual(self.router.recommend_planner(cell="other_cell")["recommended"], "umarell_u3")

    def test_u3_plans_umarells_cell_with_the_factoryos_meshes(self):
        st = self.wait(self.router.plan("umarell_u3")["job_id"])
        self.assertEqual(st["status"], "done", st)
        self.assertTrue(st["export"]["passed"])
        self.assertEqual(st["export"]["u3"]["welds"], 1)
        run = json.loads((self.root / "last_run.json").read_text())
        self.assertEqual((run["task"], run["own"], run["ops"]), ("car_door_spot_welding.json", False, ["W01"]))
        self.assertEqual(Path(run["robots"]), (self.cell / "robots").resolve())
        self.assertEqual((self.root / "third_party" / "sample-umarell").resolve(), (self.tmp / "umarell").resolve())
        plan = json.loads((self.cell / st["plan_path"]).read_text())
        self.assertEqual(plan["planner"], "umarell_u3")

    def test_u3_plans_a_changed_cell_itself(self):
        (self.cell / "scene.yaml").write_text("scene: moved\n")
        for rel in ("agentic_planner/config/robots.yaml", "agentic_planner/config/obstacles.yaml"):   # Prepare Cell's
            (self.cell / rel).parent.mkdir(parents=True, exist_ok=True)
            (self.cell / rel).write_text("x: 1\n")
        tasks = self.cell / R.TASKS_DIR
        tasks.mkdir(parents=True)
        (tasks / "mine.json").write_text('{"operations": [{"id": "W07"}, {"id": "W08"}]}')
        st = self.wait(self.router.plan("umarell_u3", task="mine")["job_id"])
        self.assertEqual(st["status"], "done", st)
        run = json.loads((self.root / "last_run.json").read_text())
        self.assertEqual((run["task"], run["own"], run["ops"]), ("factoryos_task.json", True, ["W07", "W08"]))
        # back to umarell's layout: umarell's own (patched) cell again
        (self.cell / "scene.yaml").write_text("scene: 1\n")
        st = self.wait(self.router.plan("umarell_u3")["job_id"])
        self.assertEqual(st["status"], "done", st)
        run = json.loads((self.root / "last_run.json").read_text())
        self.assertEqual((run["own"], run["ops"]), (False, ["W01"]))

    def test_u3_jobs_on_one_cell_take_turns(self):
        tasks = self.cell / R.TASKS_DIR
        tasks.mkdir(parents=True)
        (tasks / "slow.json").write_text('{"operations": [{"id": "SLOW1"}]}')
        for rel in ("agentic_planner/config/robots.yaml", "agentic_planner/config/obstacles.yaml"):   # Prepare Cell's
            (self.cell / rel).parent.mkdir(parents=True, exist_ok=True)
            (self.cell / rel).write_text("x: 1\n")
        a = self.router.plan("umarell_u3", task="slow")
        b = self.router.plan("umarell_u3", task="slow")
        self.assertEqual((a["status"], b["status"]), ("planning", "queued"))
        overlap, deadline = 0, time.time() + 30
        while time.time() < deadline:
            with self.router.lock:
                overlap = max(overlap, sum(1 for j in self.router.jobs.values()
                                           if j.planner == "umarell_u3" and j.status == "planning"))
            st = [self.router.plan_status(r["job_id"]) for r in (a, b)]
            if st[1]["status"] == "queued" and st[0]["status"] == "planning":
                self.assertIn("umarell_u3", st[1].get("waiting_for", ""))
            if all(x["status"] in R.Router.TERMINAL for x in st):
                break
            time.sleep(0.02)
        self.assertEqual([x["status"] for x in st], ["done", "done"], st)
        self.assertEqual(overlap, 1)

    def test_u3_second_job_on_a_cell_is_queued_before_a_worker_took_the_first(self):
        """plan's answer for the second U3 job on a cell is queued also while the first one still waits for its
        worker (a loaded host: no worker woke between the two plan calls; it answered planning, as the first)."""
        tasks = self.cell / R.TASKS_DIR
        tasks.mkdir(parents=True)
        (tasks / "slow.json").write_text('{"operations": [{"id": "SLOW1"}]}')
        for rel in ("agentic_planner/config/robots.yaml", "agentic_planner/config/obstacles.yaml"):   # Prepare Cell's
            (self.cell / rel).parent.mkdir(parents=True, exist_ok=True)
            (self.cell / rel).write_text("x: 1\n")
        cv = self.router.cv
        cv.notify_all = lambda: None                     # the workers sleep through both submissions
        try:
            a = self.router.plan("umarell_u3", task="slow")
            b = self.router.plan("umarell_u3", task="slow")
        finally:
            del cv.notify_all
            with cv:
                cv.notify_all()
        self.assertEqual((a["status"], b["status"]), ("planning", "queued"))
        st = [self.wait(r["job_id"]) for r in (a, b)]
        self.assertEqual([x["status"] for x in st], ["done", "done"], st)

    def _u3_cmd(self, job_id):
        cmd = self.router.jobs[job_id].cmd
        i = cmd.index("--")
        return cmd[:i], cmd[i + 1:]

    def test_u3_default_speed_is_max(self):
        """No options: U3 at the robots' limits (--speed max) and the full-profile export."""
        r = self.router.plan("umarell_u3")
        self.assertEqual((r["profile"], r["speed"]), ("full", "max"))
        u3, export = self._u3_cmd(r["job_id"])
        self.assertEqual(u3[u3.index("--speed") + 1], "max")
        self.assertEqual(export[export.index("--profile") + 1], "full")
        st = self.wait(r["job_id"])
        self.assertEqual(st["status"], "done", st)
        self.assertEqual(json.loads((self.root / "last_run.json").read_text())["speed"], "max")
        self.assertEqual(st["speed"], "max")
        self.assertEqual(st["export"]["u3"]["speed"], {"preset": "max"})
        # explicit max and profile full alone: the same
        for kw in ({"options": {"speed": "max"}}, {"profile": "full"}):
            r = self.router.plan("umarell_u3", **kw)
            self.assertEqual((r["profile"], r["speed"]), ("full", "max"), kw)
            self.wait(r["job_id"])
        self.assertEqual(self.router.describe_planner("umarell_u3")["default_profile"], "full")

    def test_u3_normal_speed_is_the_opt_out(self):
        """options.speed normal (alias default) or profile umarell: U3's planning speeds in U3's own timing."""
        for kw in ({"options": {"speed": "normal"}}, {"options": {"speed": "default"}}, {"profile": "umarell"}):
            r = self.router.plan("umarell_u3", **kw)
            self.assertEqual((r["profile"], r["speed"]), ("umarell", "normal"), kw)
            u3, export = self._u3_cmd(r["job_id"])
            self.assertNotIn("--speed", u3)
            self.assertEqual(export[export.index("--profile") + 1], "umarell")
            st = self.wait(r["job_id"])
            self.assertEqual(st["status"], "done", st)
            self.assertEqual(json.loads((self.root / "last_run.json").read_text())["speed"], "default")
            self.assertEqual(st["speed"], "normal")
        # max in the umarell profile (U3's own timing) is refused; profile full with speed normal re-times only
        with self.assertRaises(R.ToolError) as e:
            self.router.plan("umarell_u3", profile="umarell", options={"speed": "max"})
        self.assertIn("full profile only", str(e.exception))
        r = self.router.plan("umarell_u3", profile="full", options={"speed": "normal"})
        self.assertEqual((r["profile"], r["speed"]), ("full", "normal"))
        self.assertNotIn("--speed", self._u3_cmd(r["job_id"])[0])
        self.wait(r["job_id"])

    def test_u3_speed_is_checked(self):
        with self.assertRaises(R.ToolError) as e:
            self.router.plan("umarell_u3", options={"speed": "ludicrous"})
        self.assertIn("unknown options.speed", str(e.exception))
        with self.assertRaises(R.ToolError) as e:
            self.router.plan("hier_la_fu", options={"speed": "max"})
        self.assertIn("umarell_u3's", str(e.exception))

    def test_u3_max_speed_needs_a_cell_planner_that_has_it(self):
        main = self.tmp / "src" / "umarell_fast" / "u3" / "__main__.py"
        main.write_text(FAKE_U3.replace("SPEED_PRESETS", "PRESETS"))
        st = self.wait(self.router.plan("umarell_u3", options={"speed": "max"})["job_id"])
        self.assertEqual(st["status"], "failed", st)
        self.assertIn("no --speed", (st.get("error") or "") + json.dumps(st))

    def test_u3_without_a_plan_publishes_nothing(self):
        tasks = self.cell / R.TASKS_DIR
        tasks.mkdir(parents=True)
        (tasks / "bad.json").write_text('{"operations": [{"id": "FAIL"}]}')
        st = self.wait(self.router.plan("umarell_u3", task="bad")["job_id"])
        self.assertEqual(st["status"], "failed", st)
        self.assertIn("U3 found no complete plan", st["error"])
        self.assertFalse((self.cell / R.PLANS_DIR).exists() and any((self.cell / R.PLANS_DIR).iterdir()))


class WithoutBurstTest(unittest.TestCase):
    """server.py alone, without burst.py next to it: it starts, and remote planning is off whatever FACTORYOS_BURST
    says (options.remote true is told it is not installed)."""

    def test_router_without_burst(self):
        with tempfile.TemporaryDirectory(prefix="router-no-burst-") as d:
            shutil.copy(ROUTER_DIR / "server.py", d)
            (Path(d) / "work").mkdir()
            code = (
                "import sys; from pathlib import Path; sys.path.insert(0, sys.argv[1]); import server as R\n"
                "d = Path(sys.argv[1])\n"
                "s = R.Settings(cell_planner=d, python=Path(sys.executable), umarell=d, config=None, work=d / 'work',\n"
                "               planners_file=Path(sys.argv[2]), umarell_url='http://127.0.0.1:9/mcp', workers=1,\n"
                "               umarell_workers=1)\n"
                "r = R.Router(s)\n"
                "print(R.HAVE_BURST, r.burst_mode, r._remote_choice({}), r._remote_choice({'remote': True}))\n")
            out = subprocess.run([sys.executable, "-I", "-c", code, d, str(ROUTER_DIR / "planners.json")],
                                 capture_output=True, text=True, timeout=60,
                                 env={**os.environ, "FACTORYOS_BURST": "always", "PYTHONDONTWRITEBYTECODE": "1"})
        self.assertEqual(out.returncode, 0, out.stderr)
        self.assertEqual(out.stdout.strip(), "False off {} {'use': False, 'why': 'options.remote true', 'fallback': "
                         "'remote planning is not available (not installed on this host: no burst.py)'}")


if __name__ == "__main__":
    unittest.main()
