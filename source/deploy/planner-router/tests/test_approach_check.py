"""approach_check.py without umarell: the stroke order, the outcome line, the blocker's names.

    python3 -I -m unittest discover -s deploy/planner-router/tests
"""
from __future__ import annotations

import io
import json
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import approach_check as A  # noqa: E402

PAIR = {"robot": "robot1", "weld": "W02", "position": [-0.927, -1.6, 0.75], "approach_vector": [0.978, 0.148, -0.144],
        "orientation": {"x": 0.0692, "y": -0.9922, "z": -0.0692, "w": 0.0774}}


class CheckPairTest(unittest.TestCase):
    def setUp(self):
        self._env, self._stroke = A.robot_env, A.stroke_check
        A.robot_env = lambda job, ns: {"ns": ns}
        self.tried = []

    def tearDown(self):
        A.robot_env, A.stroke_check = self._env, self._stroke

    def stub(self, clear_at=None):
        def stroke(env, pair, s):
            self.tried.append(s)
            if s == clear_at:
                return {"stroke_m": s, "clear": True, "search": {"status": "chosen", "spin_deg": 90.0}}
            return {"stroke_m": s, "clear": False, "reason_code": "goal_in_collision", "reason": f"blocked at {s}",
                    "blocker": {"type": "box", "object_id": f"f{s}", "link": "link_5", "step": 2, "steps": 6,
                                "clearance_m": -0.009},
                    "search": {"status": "none_usable", "candidates": 407}}
        A.stroke_check = stroke

    def test_strokes_in_u3_order_until_one_is_clear(self):
        self.stub(clear_at=0.2)
        line = A.check_pair({"cell_id": "c"}, PAIR)
        self.assertEqual(self.tried, [0.12, 0.2])
        self.assertTrue(line["clear"])
        self.assertEqual(line["stroke_m"], 0.2)
        self.assertEqual(line["search"]["spin_deg"], 90.0)
        self.assertNotIn("blocker", line)

    def test_every_stroke_blocked_reports_the_first(self):
        self.stub()
        line = A.check_pair({"cell_id": "c", "strokes_m": [0.12, 0.08]}, PAIR)
        self.assertEqual(self.tried, [0.12, 0.08])
        self.assertIs(line["clear"], False)
        self.assertIsNone(line["stroke_m"])
        self.assertEqual((line["reason_code"], line["blocker"]["object_id"]), ("goal_in_collision", "f0.12"))
        self.assertEqual([s["stroke_m"] for s in line["strokes"]], [0.12, 0.08])

    def test_an_exception_is_that_pairs_error(self):
        def boom(env, pair, s):
            raise RuntimeError("no model")
        A.stroke_check = boom
        line = A.check_pair({"cell_id": "c"}, PAIR)
        self.assertIsNone(line["clear"])
        self.assertEqual(line["reason_code"], "error")
        self.assertIn("no model", line["reason"])


class CheckChainTest(unittest.TestCase):
    """A chain is walked as the planner chains legs: the first weld from HOME (seed None), every later one after
    the lift-off out of the last CLEAR weld (U3's retreats) and warm-started from that weld's end."""

    W = [{"weld": f"W0{i}", "position": [0.0, 0.0, float(i)], "approach_vector": [1, 0, 0],
          "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}} for i in (1, 2, 3, 4)]

    def setUp(self):
        self._saved = A.robot_env, A.stroke_check, A.liftoff_check
        A.robot_env = lambda job, ns: {"ns": ns}
        self.strokes, self.lifts = [], []
        self.blocked, self.no_lift = set(), set()

        def stroke(env, pair, s, seed=None):
            self.strokes.append((pair["weld"], s, seed))
            if pair["weld"] in self.blocked:
                return {"stroke_m": s, "clear": False, "reason_code": "goal_in_collision", "reason": "blocked",
                        "blocker": {"type": "box", "object_id": "v1", "step": 3, "steps": 6}}
            return {"stroke_m": s, "clear": True, "end_q": [float(pair["position"][2])] * 6,
                    "pose": {"target": pair["position"], "approach_vector": pair["approach_vector"],
                             "orientation": pair["orientation"]}}

        def lift(env, pose, start_q, r):
            self.lifts.append((pose["target"][2], start_q[0], r))
            return pose["target"][2] not in self.no_lift

        A.stroke_check, A.liftoff_check = stroke, lift

    def tearDown(self):
        A.robot_env, A.stroke_check, A.liftoff_check = self._saved

    def test_each_weld_from_the_last_clear_ones_end(self):
        self.blocked = {"W02"}
        lines = A.check_chain({"cell_id": "c", "strokes_m": [0.12, 0.2]}, {"robot": "robot1", "welds": self.W[:3]})
        self.assertEqual([(l["weld"], l["predecessor"], l["clear"]) for l in lines],
                         [("W01", None, True), ("W02", "W01", False), ("W03", "W01", True)])
        self.assertTrue(all(l["chained"] for l in lines))
        self.assertIsNone(lines[0]["liftoff"])
        self.assertEqual(self.strokes[0], ("W01", 0.12, None), "the first weld from HOME")
        self.assertEqual([s for s in self.strokes if s[0] == "W02"], [("W02", 0.12, [1.0] * 6), ("W02", 0.2, [1.0] * 6)])
        self.assertEqual(self.strokes[-1], ("W03", 0.12, [1.0] * 6), "W02 was skipped: W03 chains from W01")
        self.assertEqual(lines[1]["reason_code"], "goal_in_collision")
        self.assertEqual(lines[1]["blocker"]["object_id"], "v1")
        self.assertEqual(self.lifts[0], (1.0, 1.0, 0.12), "the lift-off out of W01 at the stroke first")

    def test_blocked_liftoff_tries_u3_retreats_then_refuses(self):
        self.no_lift = {2.0}
        lines = A.check_chain({"cell_id": "c"}, {"robot": "robot1", "welds": self.W[:3]})
        self.assertEqual([r for _, _, r in self.lifts if _ == 2.0], [0.12, 0.08, 0.05, 0.03])
        self.assertEqual((lines[2]["clear"], lines[2]["reason_code"], lines[2]["predecessor"]),
                         (False, "liftoff_blocked", "W02"))
        self.assertIn("lift-off out of W02", lines[2]["reason"])
        self.assertEqual(lines[2]["liftoff"]["clear"], False)

    def test_run_emits_pairs_and_chains(self):
        self._saved_pair = A.check_pair
        A.check_pair = lambda job, pair: {"robot": pair["robot"], "weld": pair["weld"], "clear": True}
        try:
            out = io.StringIO()
            job = {"cell_id": "c", "workers": 1, "pairs": [dict(PAIR)],
                   "chains": [{"robot": "robot2", "welds": self.W[:2]}, {"robot": "robot3", "welds": []}]}
            import types
            sys.modules.setdefault("agents", types.ModuleType("agents"))
            for n in ("agents.trajectory", "agents.trajectory.tools", "agents.trajectory.tools.leg_builder",
                      "agents.trajectory.tools.weld_orientation"):
                sys.modules.setdefault(n, types.ModuleType(n))
            self.assertEqual(A.run(job, out), 0)
        finally:
            A.check_pair = self._saved_pair
            for n in ("agents.trajectory.tools.weld_orientation", "agents.trajectory.tools.leg_builder",
                      "agents.trajectory.tools", "agents.trajectory", "agents"):
                if not hasattr(sys.modules.get(n), "__file__"):
                    sys.modules.pop(n, None)
        got = [json.loads(l) for l in out.getvalue().splitlines()]
        self.assertEqual([(g.get("robot"), g.get("weld"), g.get("chained", False)) for g in got[:-1]],
                         [("robot1", "W02", False), ("robot2", "W01", True), ("robot2", "W02", True)])
        self.assertEqual(got[-1], {"done": True})


class BlockerTest(unittest.TestCase):
    MODEL = SimpleNamespace(frames=[SimpleNamespace(name=f"frame{i}") for i in range(20)])

    def test_names(self):
        why = {"cause": "collision", "step": 1, "steps": 6,
               "collision": {"type": "box", "object_id": "body_weld_3_v0005", "link_index": 16,
                             "clearance_m": -0.0252}}
        self.assertEqual(A.blocker(self.MODEL, why), {"type": "box", "object_id": "body_weld_3_v0005",
                                                      "link": "frame16", "is_workpiece": False, "step": 1,
                                                      "steps": 6, "clearance_m": -0.0252})
        self.assertEqual(A.link_name(self.MODEL, {"type": "tool_penetration", "object_id": "x"}), "the tool")
        self.assertEqual(A.link_name(self.MODEL, {"type": "self", "object_id": "link_3 vs link_6"}), "link_3 vs link_6")
        self.assertEqual(A.blocker(self.MODEL, {"cause": "ik", "step": 4, "steps": 6}),
                         {"type": "ik", "is_workpiece": False, "step": 4, "steps": 6})


class RunTest(unittest.TestCase):
    def test_job_errors_and_empty_jobs(self):
        out = io.StringIO()
        self.assertEqual(A.run({"pairs": [PAIR]}, out), 1)
        self.assertIn("cell_id", json.loads(out.getvalue())["error"])
        out = io.StringIO()
        self.assertEqual(A.run({"cell_id": "c", "pairs": []}, out), 0)
        self.assertEqual(json.loads(out.getvalue()), {"done": True})


if __name__ == "__main__":
    unittest.main()
