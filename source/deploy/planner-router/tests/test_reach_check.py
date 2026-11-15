"""Unit tests of the Robo Fitter's reach check script (standard library only):

    python3 -I -m unittest discover -s deploy/planner-router/tests

The IK itself runs only where the cell planner is: set FOS_PLANNER_PYTHON (its venv's python) and
FOS_PLANNER_SRC (its sources) to also run jam2's robot on its welds; skipped otherwise.
"""
from __future__ import annotations

import json
import math
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROUTER_DIR = Path(__file__).resolve().parents[1]
REPO = ROUTER_DIR.parents[1]
sys.path.insert(0, str(ROUTER_DIR))
import reach_check as RC  # noqa: E402

URDF = REPO / "crates/factoryos-backend/tests/fixtures/jam2/kr240r2900__sticktool/kr240r2900__sticktool.urdf"
CATALOGUE_URDF = REPO / "assets/catalogue/urdf-dressed-robots/kr240r2900__sticktool/kr240r2900__sticktool.urdf"
TCP = {"tip_link": "like_kr240r2900_link_6", "offset": [-0.9092, 0.0056, 0.0005], "axis": [-1.0, 0.0, 0.0]}


def job(urdf: Path, mode="quick", tilt=(0.0, 0.0), welds=None) -> dict:
    return {"mode": mode,
            "robots": [{"id": "r1", "urdf": str(urdf), "base_xyz": [2.5, 0.0, 1.0], "yaw_deg": 180.0,
                        "tilt_bc_deg": list(tilt), "home_deg": [45, -90, 90, 0, 0, 0], "tcp": TCP}],
            "welds": welds if welds is not None else [{"id": "W01", "position": [-0.5, -0.5, 2.0], "approach": [0, 0, -1]},
                                                      {"id": "W02", "position": [0.5, -0.5, 2.0], "approach": [0, 0, -1]}],
            "floor_z": 0.0, "tolerances": {"position_m": 0.01, "tool_axis_deg": 5.0}}


def matvec(R, v):
    return [sum(R[i][k] * v[k] for k in range(3)) for i in range(3)]


class ReachCheckTest(unittest.TestCase):
    def test_reasons(self):
        self.assertEqual(RC.normalise_reason("self_collision(0.0012)"), "self_collision")
        self.assertEqual(RC.normalise_reason("floor(-0.010)"), "floor")
        self.assertEqual(RC.normalise_reason("umarell_floor"), "floor")
        self.assertEqual(RC.normalise_reason("pose_tolerance"), "pose_tolerance")
        self.assertEqual(RC.normalise_reason("something new"), "error")
        self.assertEqual(RC.normalise_reasons(["floor(-0.1)", "umarell_floor", "joint_limits"]), ["floor", "joint_limits"])

    def test_scene(self):
        scene = RC.scene_dict(job(URDF))
        self.assertEqual(scene["schema_version"], 2)
        r = scene["robots"][0]
        self.assertEqual(r["model"], "kr240r2900")
        self.assertEqual(r["base"], {"xyz": [2.5, 0.0, 1.0], "yaw_deg": 180.0})
        self.assertEqual(r["tool"]["tcp_offset"], TCP["offset"])
        self.assertEqual(r["tool"]["axis"], [-1.0, 0.0, 0.0])
        pairs = r["collision"]["disabled_pairs"]
        self.assertIn(["like_kr240r2900_base_link", "like_kr240r2900_link_1"], pairs)
        self.assertIn(["like_kr240r2900_link_5", "like_kr240r2900_link_6"], pairs)
        spheres = r["collision"]["tool_spheres"]
        self.assertEqual(spheres["frame"], "like_kr240r2900_link_6")
        first, last = spheres["spheres"][0], spheres["spheres"][-1]
        self.assertAlmostEqual(math.dist(first[:3], [0, 0, 0]), 0.15, places=6)
        self.assertAlmostEqual(last[0], -0.9092, places=4)
        self.assertTrue(all(s[3] == 0.07 for s in spheres["spheres"]))
        self.assertEqual(scene["obstacles"]["floor"]["z"], 0.0)
        self.assertEqual(scene["tolerances"]["position_m"], 0.01)

    def test_tcp_rotation_points_the_tool_along_the_approach(self):
        for axis, approach in (([-1, 0, 0], [0, 0, -1]), ([0, 0, 1], [1, 0, 0]), ([-1, 0, 0], [0.6, 0, -0.8])):
            for spin in (0.0, 180.0, 37.0):
                R = RC.tcp_rotation(axis, approach, spin)
                got = matvec(R, axis)
                for g, w in zip(got, RC._norm(approach)):
                    self.assertAlmostEqual(g, w, places=9)
                det = (R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1]) - R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0])
                       + R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]))
                self.assertAlmostEqual(det, 1.0, places=9)

    def run_script(self, payload: dict, python=sys.executable, src: str = "/nonexistent", timeout=600):
        proc = subprocess.run([python, "-I", str(ROUTER_DIR / "reach_check.py"), "--planner-src", src],
                              input=json.dumps(payload), capture_output=True, text=True, timeout=timeout)
        return proc.returncode, [json.loads(line) for line in proc.stdout.splitlines() if line.strip()]

    def test_tilted_bases_are_reported_without_the_ik(self):
        code, lines = self.run_script(job(URDF, tilt=(0.0, 5.0)))
        self.assertEqual(code, 0)
        self.assertEqual([l.get("reasons") for l in lines[:-1]], [["base_tilt_unsupported"]] * 2)
        self.assertEqual(lines[-1], {"done": True})

    def test_without_trajgen_the_job_fails_cleanly(self):
        code, lines = self.run_script(job(URDF), src=tempfile.mkdtemp())
        self.assertEqual(code, 1)
        self.assertIn("error", lines[-1])

    @unittest.skipUnless(os.environ.get("FOS_PLANNER_PYTHON") and os.environ.get("FOS_PLANNER_SRC"),
                         "set FOS_PLANNER_PYTHON and FOS_PLANNER_SRC to run the cell planner's IK")
    def test_jam2_with_the_planner_ik(self):
        # The full dressed URDF (collision hulls): r1 at x = 2.5 reaches W02 (near), not W01 (57 mm out of reach).
        urdf = Path(os.environ.get("FOS_KR240_URDF") or CATALOGUE_URDF)
        for mode in ("quick", "full"):
            code, lines = self.run_script(job(urdf, mode=mode), python=os.environ["FOS_PLANNER_PYTHON"],
                                          src=os.environ["FOS_PLANNER_SRC"])
            self.assertEqual(code, 0, lines)
            by_weld = {l["weld"]: l for l in lines if "weld" in l}
            self.assertFalse(by_weld["W01"]["reachable"], by_weld)
            self.assertTrue(by_weld["W02"]["reachable"], by_weld)
            self.assertEqual(len(by_weld["W02"]["q_deg"]), 6)
            self.assertLess(by_weld["W02"]["err"]["pos_m"], 0.01)
            self.assertEqual(lines[-1], {"done": True})


if __name__ == "__main__":
    unittest.main()
