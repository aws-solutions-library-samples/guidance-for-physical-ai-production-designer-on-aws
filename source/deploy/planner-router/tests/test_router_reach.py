"""The planner router's Robo Fitter tools (check_reachability, suggest_layout) against a fake backend:

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

REPORT = {
    "scene_rev": 7, "source": "product", "mode": "quick", "method": "ik", "running": False, "done": 2, "total": 2,
    "welds": [
        {"weld": {"id": "bug_cube/W01", "label": "W01", "position": [-0.5, -0.5, 2.0], "approach": [0, 0, -1]},
         "attempts": [{"robot_id": "r1", "reachable": False, "joints_deg": [], "margin_m": -0.053,
                       "reasons": ["out_of_reach"], "detail": "0.053 m outside the dome"},
                      {"robot_id": "r2", "reachable": True, "joints_deg": [14.0, -74.3, 88.4, 0.0, 75.8, -76.0],
                       "margin_m": 0.82, "reasons": [], "detail": ""}],
         "reached_by": ["r2"], "status": "one_robot"}],
    "suggestions": [{"robot_id": "r1", "text": "For a second robot on W01: move r1 by 0.12 m"}],
    "notes": ["the planner IK does not check fixtures"], "error": None, "elapsed_s": 3.2,
}


FULL = {
    "scene_rev": 8, "source": "cell_program", "mode": "full", "method": "ik", "running": False, "done": 4, "total": 4,
    "welds": [
        {"weld": {"id": "W01", "label": "W01", "task": "spot", "position": [-0.927, -1.6, 0.75],
                  "approach": [0.978, 0.148, -0.144]},
         "attempts": [{"robot_id": "gun", "reachable": True, "joints_deg": [1.0, -60.0, 120.0, -85.0, -87.0, -63.0],
                       "margin_m": 0.8, "reasons": [], "detail": "IK 2.4 s"},
                      {"robot_id": "gun_2", "reachable": True, "joints_deg": [2.0, -61.0, 121.0, 85.0, -87.0, 59.0],
                       "margin_m": 0.5, "reasons": [], "detail": "IK 2.0 s"}],
         "reached_by": ["gun", "gun_2"], "status": "all_robots"},
        {"weld": {"id": "W02", "label": "W02", "task": "spot", "position": [0.9, -1.6, 0.75], "approach": [-1, 0, 0]},
         "attempts": [{"robot_id": "gun", "reachable": False, "joints_deg": [], "margin_m": -1.3,
                       "reasons": ["out_of_reach"], "detail": ""},
                      {"robot_id": "gun_2", "reachable": True, "joints_deg": [3.0, -50.0, 110.0, 90.0, -90.0, 60.0],
                       "margin_m": 0.4, "reasons": [], "detail": ""}],
         "reached_by": ["gun_2"], "status": "one_robot"}],
    "suggestions": [], "notes": [], "error": None, "elapsed_s": 10.0,
}
W01_ORI = {"x": 0.0692, "y": -0.9922, "z": -0.0692, "w": 0.0774}
CELL_YAML = """# Cell manifest (Prepare Cell)
name: welding_cell_2_small
cell_id: welding_cell_2_small
robots:
  - id: robot1
    scene_id: gun
    namespace: arms
  - id: robot2
    scene_id: gun_2
    namespace: arms
"""


class FakeBackend:
    def __init__(self):
        self.calls = []
        self.report = REPORT
        self.cells = None
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
                if self.path == "/api/v1/reach/check":
                    return self._send(200, json.dumps(outer.report))
                if self.path == "/api/v1/reach/suggest":
                    if body.get("write_as") == "jam2.yaml":
                        return self._send(409, "jam2.yaml exists in scenes/: scenes are never overwritten",
                                          "text/plain")
                    return self._send(200, json.dumps({"scene": {"name": "jam2-fit"}, "report": REPORT,
                                                       "moves": REPORT["suggestions"], "written": body.get("write_as")}))
                return self._send(404, "no route", "text/plain")

            def do_GET(self):
                if self.path == "/api/v1/cell/cells" and outer.cells is not None:
                    return self._send(200, json.dumps(outer.cells))
                return self._send(404, "no route", "text/plain")

        self.httpd = ThreadingHTTPServer(("127.0.0.1", 0), H)
        self.url = f"http://127.0.0.1:{self.httpd.server_address[1]}"
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def close(self):
        self.httpd.shutdown()
        self.httpd.server_close()


class ReachBase(RouterBase):
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


class ReachToolsTest(ReachBase):

    def test_tools_are_listed(self):
        _, r, _ = R.handle_rpc(self.router, {"jsonrpc": "2.0", "id": 2, "method": "tools/list"})
        tools = {t["name"]: t for t in r["result"]["tools"]}
        self.assertEqual(set(tools["check_reachability"]["inputSchema"]["properties"]),
                         {"scene", "cell", "mode", "source", "show_in_fitter", "chained"})
        self.assertEqual(set(tools["suggest_layout"]["inputSchema"]["properties"]),
                         {"scene", "cell", "max_move_m", "write_as"})

    def test_check_reachability_is_forwarded(self):
        error, result = self.call("check_reachability", scene="jam2.yaml", mode="full", show_in_fitter=True)
        self.assertFalse(error, result)
        self.assertEqual(self.backend.calls, [("/api/v1/reach/check", {"scene_file": "jam2.yaml", "mode": "full",
                                                                       "source": "auto", "show_in_fitter": True})])
        self.assertEqual(result["report"]["scene_rev"], 7)
        table = result["table"]
        self.assertIn("1 welds (product, ik, quick, 3 s): 0 green, 1 orange, 0 red", table)
        self.assertIn("bug_cube/W01 [orange]: r1: out_of_reach (dome margin -0.053 m); r2: reached (q° 14.0, -74.3", table)
        self.assertIn("suggestion: For a second robot on W01", table)

    def test_bad_arguments_are_tool_errors(self):
        for args in ({}, {"scene": "jam2.yaml", "mode": "slow"}, {"cell": "../x"}):
            error, result = self.call("check_reachability", **args)
            self.assertTrue(error, args)
            self.assertIn("error", result)
        self.assertEqual(self.backend.calls, [])

    def test_suggest_layout_never_overwrites(self):
        error, result = self.call("suggest_layout", scene="jam2.yaml", write_as="jam2-fit.yaml", max_move_m=0.5)
        self.assertFalse(error, result)
        self.assertEqual(self.backend.calls[-1], ("/api/v1/reach/suggest", {"scene_file": "jam2.yaml", "max_move_m": 0.5,
                                                                            "write_as": "jam2-fit.yaml"}))
        self.assertEqual(result["written"], "jam2-fit.yaml")
        self.assertEqual(result["moves"], ["For a second robot on W01: move r1 by 0.12 m"])
        error, result = self.call("suggest_layout", scene="jam2.yaml", write_as="jam2.yaml")
        self.assertTrue(error)
        self.assertIn("never overwritten", result["error"])
        error, result = self.call("suggest_layout", scene="jam2", write_as="jam2-fit2")
        self.assertFalse(error, result)
        self.assertEqual(self.backend.calls[-1][1]["write_as"], "jam2-fit2", "a scene name")
        error, result = self.call("suggest_layout", scene="jam2.yaml", write_as="../cells/x.yaml")
        self.assertTrue(error)
        self.assertEqual(len(self.backend.calls), 3, "a path is refused before the backend")



class ApproachCheckTest(ReachBase):
    """mode full on a prepared cell: the router re-checks every reached weld x robot with the planner's approach
    predicate (approach_check.py, stubbed here) and folds its refusals into the report."""

    def setUp(self):
        super().setUp()
        self.backend.report = FULL
        ap = self.cell / "agentic_planner"
        (ap / "tasks").mkdir(parents=True, exist_ok=True)
        (ap / "cell.yaml").write_text(CELL_YAML)
        (ap / "tasks" / "spot.json").write_text(json.dumps({"operations": [
            {"id": "W01", "position": [-0.927, -1.6, 0.75], "approach_vector": [0.978, 0.148, -0.144],
             "orientation": W01_ORI}]}))
        self.router.s.umarell_python = self.tmp / "python"
        self.router.s.umarell_python.write_text("")
        self.jobs = []

        def stub(root, job):
            self.jobs.append((root, job))
            out = []
            for p in job["pairs"]:
                if (p["robot"], p["weld"]) == ("robot2", "W01"):
                    out.append({"robot": "robot2", "weld": "W01", "clear": False, "stroke_m": None,
                                "reason_code": "goal_in_collision",
                                "reason": "the straight on-axis descent to the weld is blocked",
                                "blocker": {"type": "box", "object_id": "body_weld_3_f0026", "link": "link_5",
                                            "is_workpiece": True, "step": 2, "steps": 6, "clearance_m": -0.009},
                                "search": {"status": "none_usable", "candidates": 407},
                                "strokes": [{"stroke_m": 0.12, "clear": False}, {"stroke_m": 0.2, "clear": False},
                                            {"stroke_m": 0.08, "clear": False}], "elapsed_s": 58.0})
                else:
                    out.append({"robot": p["robot"], "weld": p["weld"], "clear": True, "stroke_m": 0.12,
                                "search": {"status": "specified_ok", "candidates": 0},
                                "strokes": [{"stroke_m": 0.12, "clear": True}], "elapsed_s": 20.0})
            return out

        self.router._run_approach_check = stub

    def test_blocked_approach_is_not_reached(self):
        error, result = self.call("check_reachability", cell=self.CID, mode="full", source="cell_program")
        self.assertFalse(error, result)
        (root, job), = self.jobs
        self.assertEqual(root, self.cell)
        self.assertEqual(job["cell_id"], self.CID)
        self.assertEqual(job["strokes_m"], [0.12, 0.2, 0.08])
        pairs = {(p["robot"], p["weld"]): p for p in job["pairs"]}
        self.assertEqual(set(pairs), {("robot1", "W01"), ("robot2", "W01"), ("robot2", "W02")})
        self.assertEqual(pairs[("robot1", "W01")]["orientation"], W01_ORI, "the task program's orientation")
        self.assertAlmostEqual(R.angle_deg(R.tool_axis(pairs[("robot2", "W02")]["orientation"]), [-1, 0, 0]), 0.0,
                               places=3, msg="no program operation: derived from the approach")
        w01, w02 = result["report"]["welds"]
        self.assertEqual(w01["status"], "one_robot")
        self.assertEqual(w01["reached_by"], ["gun"])
        blocked = w01["attempts"][1]
        self.assertFalse(blocked["reachable"])
        self.assertEqual(blocked["reasons"], ["approach_blocked"])
        self.assertEqual(w02["status"], "one_robot")
        ap = result["report"]["approach"]
        self.assertEqual((ap["checked"], ap["pairs"], ap["clear"], ap["blocked"]), (True, 3, 2, 1))
        table = result["table"]
        self.assertIn("2 welds (cell_program, ik, full, 10 s): 0 green, 2 orange, 0 red", table)
        self.assertIn("2 clear, 1 approach_blocked", table)
        self.assertIn("gun_2: approach_blocked: link_5 vs body_weld_3_f0026 (workpiece) at stroke step 2/6, -0.9 cm "
                      "(0.12 m stroke; 0.2, 0.08 m blocked too); tool spin/tilt searched: none of 407 poses usable",
                      table)
        self.assertIn("gun: reached (q° 1.0, -60.0, 120.0, -85.0, -87.0, -63.0), approach clear (0.12 m stroke)", table)

    def test_every_robot_blocked_is_red(self):
        def all_blocked(root, job):
            return [{"robot": p["robot"], "weld": p["weld"], "clear": False, "reason_code": "approach_unreachable",
                     "blocker": {"type": "ik", "step": 4, "steps": 6}, "strokes": [{"stroke_m": 0.12}]}
                    for p in job["pairs"]]
        self.router._run_approach_check = all_blocked
        _, result = self.call("check_reachability", cell=self.CID, mode="full")
        self.assertEqual([w["status"] for w in result["report"]["welds"]], ["no_robot", "no_robot"])
        self.assertIn("0 green, 0 orange, 2 red", result["table"])
        self.assertIn("approach_blocked: no IK solution at the weld's lean at stroke step 4/6 (0.12 m stroke)",
                      result["table"])
        w01, w02 = result["report"]["welds"]
        self.assertNotIn("coarse voxel shell", w01["attempts"][0]["approach"]["detail"])
        self.assertIn("not a weld of the prepared cell's task program", w02["attempts"][1]["approach"]["detail"])
        self.assertEqual(result["report"]["approach"]["not_in_program"], ["W02"])

    def test_quick_mode_says_the_approach_is_not_checked(self):
        _, result = self.call("check_reachability", cell=self.CID, mode="quick")
        self.assertEqual(self.jobs, [])
        self.assertFalse(result["report"]["approach"]["checked"])
        self.assertIn("approach: not checked (quick mode skips the approach stroke", result["table"])
        self.assertEqual(result["report"]["welds"][0]["status"], "all_robots")

    def test_unprepared_or_stale_cell_is_not_checked(self):
        (self.cell / "agentic_planner" / "cell.yaml").unlink()
        _, result = self.call("check_reachability", cell=self.CID, mode="full")
        self.assertIn("is not prepared", result["report"]["approach"]["why"])
        (self.cell / "agentic_planner" / "cell.yaml").write_text(CELL_YAML)
        self.backend.cells = {"entries": [{"name": self.CID, "prepared_stale": True, "stale_reason": "scene changed"}]}
        _, result = self.call("check_reachability", cell=self.CID, mode="full")
        self.assertIn("stale (scene changed", result["report"]["approach"]["why"])
        self.assertIn("approach: not checked", result["table"])
        self.assertEqual(self.jobs, [])

    def test_a_scene_without_a_cell_is_not_checked(self):
        _, result = self.call("check_reachability", scene="jam2.yaml", mode="full")
        self.assertEqual(self.jobs, [])
        self.assertIn("no cell's layout", result["report"]["approach"]["why"])
        self.assertFalse((self.tmp / "cells" / "jam2").exists(), "the check never makes a cell")

    def test_a_scene_of_a_prepared_cell_is_checked(self):
        R.write_json_atomic(self.cell / R.MANIFEST, {"format": 2, "kind": "reference", "fitted_scene": "jam2.yaml"})
        _, result = self.call("check_reachability", scene="jam2.yaml", mode="full")
        self.assertTrue(result["report"]["approach"]["checked"], result["report"]["approach"])
        self.assertEqual(len(self.jobs), 1)
        _, result = self.call("check_reachability", scene=str(self.router.fitted_dir() / "jam2" / "scene.yaml"), mode="full")
        self.assertTrue(result["report"]["approach"]["checked"], "the Fitter's prompt passes the scene's path")
        _, result = self.call("check_reachability", scene=str(self.router.fitted_dir() / "jam2.yaml"), mode="full")
        self.assertTrue(result["report"]["approach"]["checked"], "a flat file of the old layout")
        _, result = self.call("check_reachability", scene="jam2", mode="full")
        self.assertTrue(result["report"]["approach"]["checked"], "by name")
        _, result = self.call("check_reachability", scene=str(self.cell / "scene.yaml"), mode="full")
        self.assertTrue(result["report"]["approach"]["checked"], "a cell's own scene.yaml")
        _, result = self.call("check_reachability", scene="jam2.yaml", cell="other_cell", mode="full")
        self.assertIn("is not cell other_cell's layout", result["report"]["approach"]["why"])
        self.assertEqual(len(self.jobs), 5)


ROBOTS_YAML = """# robots.yaml (Prepare Cell)
robots:
    robot1:
      model: kr240r2900
      base_position: [-2.6, -1.5, 0.0]   # m
      link_collision:
        spheres: [[0.0, 0.0, 0.1]]
    robot2:
      model: kr240r2900
      base_position: [2.6, -1.5, 0.0]
planning:
  max_velocity_scaling: 0.7
"""
CHAIN_OPS = [  # the program's order: W03 first, then W01, W02 (W02 is approached in -x: the right side's)
    {"id": "W03", "position": [-0.9, -0.9, 0.7], "approach_vector": [1, 0, 0], "orientation": W01_ORI},
    {"id": "W01", "position": [-0.927, -1.6, 0.75], "approach_vector": [0.978, 0.148, -0.144], "orientation": W01_ORI},
    {"id": "W02", "position": [0.9, -1.6, 0.75], "approach_vector": [-1, 0, 0], "orientation": W01_ORI},
]
CHAIN_FULL = {**FULL, "welds": FULL["welds"] + [
    {"weld": {"id": "W03", "label": "W03", "task": "spot", "position": [-0.9, -0.9, 0.7], "approach": [1, 0, 0]},
     "attempts": [{"robot_id": "gun", "reachable": True, "joints_deg": [0.0] * 6, "margin_m": 0.7, "reasons": [],
                   "detail": ""},
                  {"robot_id": "gun_2", "reachable": True, "joints_deg": [0.0] * 6, "margin_m": 0.3, "reasons": [],
                   "detail": ""}],
     "reached_by": ["gun", "gun_2"], "status": "all_robots"}]}


class ChainedCheckTest(ReachBase):
    """chained: each weld also from its predecessor's end (approach_check.py's chains, stubbed): the chains the
    router builds (program order on the nearest clear robot, or a plan's order) and how a chained-only failure is
    reported."""

    def setUp(self):
        super().setUp()
        self.backend.report = json.loads(json.dumps(CHAIN_FULL))
        ap = self.cell / "agentic_planner"
        (ap / "tasks").mkdir(parents=True, exist_ok=True)
        (ap / "config").mkdir(parents=True, exist_ok=True)
        (ap / "cell.yaml").write_text(CELL_YAML)
        (ap / "config" / "robots.yaml").write_text(ROBOTS_YAML)
        (ap / "tasks" / "spot.json").write_text(json.dumps({"operations": CHAIN_OPS}))
        self.router.s.umarell_python = self.tmp / "python"
        self.router.s.umarell_python.write_text("")
        self.jobs = []

        def stub(root, job):
            self.jobs.append(json.loads(json.dumps(job)))
            if job.get("pairs"):          # from HOME: everything the IK reached is clear
                return [{"robot": p["robot"], "weld": p["weld"], "clear": True, "stroke_m": 0.12,
                         "strokes": [{"stroke_m": 0.12, "clear": True}]} for p in job["pairs"]]
            out = []
            for c in job["chains"]:
                prev = None
                for i, w in enumerate(c["welds"]):
                    line = {"robot": c["robot"], "weld": w["weld"], "chained": True, "index": i, "predecessor": prev}
                    if (c["robot"], w["weld"]) == ("robot1", "W01"):
                        line.update(clear=False, reason_code="goal_in_collision",
                                    reason="the straight on-axis descent to the weld is blocked",
                                    blocker={"type": "box", "object_id": "body_weld_3_v0011", "link": "link_3",
                                             "step": 4, "steps": 6, "clearance_m": -0.019},
                                    liftoff={"clear": True, "retreat_m": 0.12})
                    else:
                        line.update(clear=True, stroke_m=0.12)
                        prev = w["weld"]
                    out.append(line)
            return out

        self.router._run_approach_check = stub

    def test_program_order_on_the_nearest_clear_robot(self):
        error, result = self.call("check_reachability", cell=self.CID, mode="full", chained="program")
        self.assertFalse(error, result)
        self.assertEqual(len(self.jobs), 2, "from HOME first, then the chains")
        chains = self.jobs[1]["chains"]
        self.assertNotIn("pairs", self.jobs[1])
        self.assertEqual(self.jobs[1]["retreats_m"], [0.08, 0.05, 0.03])
        self.assertEqual([(c["robot"], [w["weld"] for w in c["welds"]]) for c in chains],
                         [("robot1", ["W03", "W01"]), ("robot2", ["W02"])])
        w = chains[0]["welds"][1]
        self.assertEqual((w["position"], w["approach_vector"], w["orientation"]),
                         ([-0.927, -1.6, 0.75], [0.978, 0.148, -0.144], W01_ORI), "the program's operation")
        report = result["report"]
        ch = report["approach"]["chained"]
        self.assertEqual((ch["checked"], ch["order"], ch["clear"], ch["blocked_chained"], ch["liftoff_blocked_chained"]),
                         (True, "program", 2, 1, 0))
        self.assertEqual(ch["failures"], [{"weld": "W01", "robot": "gun", "robot_ns": "robot1", "predecessor": "W03",
                                           "reason": "approach_blocked_chained"}])
        w01 = next(x for x in report["welds"] if x["weld"]["id"] == "W01")
        self.assertEqual(w01["status"], "all_robots", "clear from HOME: the status stays")
        att = w01["attempts"][0]
        self.assertTrue(att["reachable"])
        self.assertEqual((att["chained"]["reason"], att["chained"]["predecessor"]), ("approach_blocked_chained", "W03"))
        self.assertEqual(w01["chained_blocked"]["robot"], "gun")
        table = result["table"]
        self.assertIn("chained (program: the task program's order; each weld on the nearest robot whose approach is "
                      "clear from HOME): 3 welds, each from its predecessor's end: 2 clear, 1 approach_blocked_chained, "
                      "0 liftoff_blocked_chained", table)
        self.assertIn("  robot1: W03 -> W01", table)
        self.assertIn("chained gun approach_blocked_chained after W03: link_3 vs body_weld_3_v0011 at stroke step 4/6, "
                      "-1.9 cm (clear from HOME)", table)
        self.assertIn("chained gun from HOME: clear (0.12 m stroke)", table)

    def test_a_plans_order(self):
        plans = self.cell / "agentic_planner" / "plans"
        plans.mkdir(parents=True, exist_ok=True)

        def leg(ns, op, kind="weld"):
            return {"type": "ExecuteTrajectory", "ports": {"robot_ns": ns, "operation_id": op, "operation_type": kind}}

        tree = {"tree_id": "bt_7_x", "behaviour_tree": {"root": {"type": "Sequence", "children": [
            {"type": "Parallel", "children": [leg("robot1", "W01"), leg("robot2", "W02")]},
            leg("robot1", "W01_HOME", "move"), leg("robot1", "W03")]}}}
        (plans / "bt_7_x.json").write_text(json.dumps(tree))
        _, result = self.call("check_reachability", cell=self.CID, mode="full", chained="bt_7_x")
        chains = self.jobs[1]["chains"]
        self.assertEqual([(c["robot"], [w["weld"] for w in c["welds"]]) for c in chains],
                         [("robot1", ["W01", "W03"]), ("robot2", ["W02"])])
        self.assertEqual(result["report"]["approach"]["chained"]["how"], "plan bt_7_x's order per robot")
        self.assertEqual(R.plan_weld_order(tree), {"robot1": ["W01", "W03"], "robot2": ["W02"]})
        _, result = self.call("check_reachability", cell=self.CID, mode="full", chained="latest")
        self.assertEqual(result["report"]["approach"]["chained"]["order"], "latest")
        self.assertEqual(len(self.jobs[3]["chains"]), 2)
        _, result = self.call("check_reachability", cell=self.CID, mode="full", chained="bt_9_none")
        self.assertFalse(result["report"]["approach"]["chained"]["checked"])
        self.assertIn("no plan bt_9_none", result["table"])

    def test_chained_needs_mode_full(self):
        error, result = self.call("check_reachability", cell=self.CID, mode="quick", chained="program")
        self.assertTrue(error)
        self.assertIn("needs mode full", result["error"])
        error, result = self.call("check_reachability", cell=self.CID, mode="full", chained="../x")
        self.assertTrue(error)
        self.assertEqual(self.jobs, [])

    def test_robot_bases_and_liftoff_blocked(self):
        self.assertEqual(R.robot_bases(self.cell), {"robot1": [-2.6, -1.5, 0.0], "robot2": [2.6, -1.5, 0.0]})
        report = json.loads(json.dumps(CHAIN_FULL))
        lines = [{"robot": "robot1", "weld": "W03", "chained": True, "predecessor": None, "clear": True,
                  "stroke_m": 0.12},
                 {"robot": "robot1", "weld": "W01", "chained": True, "predecessor": "W03", "clear": False,
                  "reason_code": "liftoff_blocked", "reason": "the on-axis lift-off out of W03 is blocked",
                  "liftoff": {"clear": False}},
                 {"robot": "robot2", "weld": "W02", "chained": True, "predecessor": "W01", "clear": False,
                  "reason_code": "goal_in_collision"}]
        report["welds"][1]["attempts"][1]["reachable"] = False       # W02 robot2 blocked from HOME too
        counts = R.apply_chained(report, lines, {"robot1": "gun", "robot2": "gun_2"})
        self.assertEqual((counts["clear"], counts["liftoff_blocked_chained"], counts["blocked_from_home_too"]),
                         (1, 1, 1))
        self.assertEqual(report["welds"][0]["chained"]["reason"], "liftoff_blocked_chained")
        self.assertIn("after W03: liftoff_blocked", report["welds"][0]["chained"]["detail"])
        self.assertEqual(report["welds"][1]["chained"]["reason"], "approach_blocked")
        self.assertNotIn("chained_blocked", report["welds"][1])


if __name__ == "__main__":
    unittest.main()
