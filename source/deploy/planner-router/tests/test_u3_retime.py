"""u3_retime: the junction repair of U3 plans (standard library only).

    python3 -I -m unittest discover -s deploy/planner-router/tests
"""
from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import u3_retime as U  # noqa: E402

A_MAX = [3.0, 3.0, 3.0, 6.0, 6.0, 8.0]
V_MAX = [2.722714, 2.722714, 2.722714, 5.759586, 5.759586, 10.73377]


def hermite_fallback(knots, velocity_scale=0.3, dt=0.08):
    """umarell's time_parameterization._hermite_spline + _hermite_fallback (the path U3 takes without TOPP-RA)."""
    n = len(knots)
    vel = [[0.0] * len(knots[0]) for _ in range(n)]
    for i in range(1, n - 1):
        vel[i] = [0.5 * (b - a) for a, b in zip(knots[i - 1], knots[i + 1])]
    out, t_acc = [], 0.0
    for i in range(n - 1):
        p0, p1, v0, v1 = knots[i], knots[i + 1], vel[i], vel[i + 1]
        seg = max(sum((b - a) ** 2 for a, b in zip(p0, p1)) ** 0.5, 0.05)
        m = max(2, int(seg / dt))
        for k in range(m):
            s = k / m
            h00, h10, h01, h11 = 2 * s**3 - 3 * s**2 + 1, s**3 - 2 * s**2 + s, -2 * s**3 + 3 * s**2, s**3 - s**2
            q = [h00 * a + h10 * c + h01 * b + h11 * d for a, b, c, d in zip(p0, p1, v0, v1)]
            out.append({"positions": q, "time_s": round((t_acc + s * seg) / velocity_scale, 4), "phase": "transit"})
        t_acc += seg
    out.append({"positions": list(knots[-1]), "time_s": round(t_acc / velocity_scale, 4), "phase": "transit"})
    return out


def liftoff_then_transit():
    """bt_1792079345_jamijam2's robot1 W04 leg, rebuilt: seven 6 mrad lift-off steps (a2 / a3 / a5) out of the
    previous weld, then one long transit (a1 = a6 +0.49 rad), then the straight approach to the weld."""
    q0 = [-0.24767, -1.29936, 1.54764, 0.0, 1.32251, -0.24767]
    knots = [q0]
    for i in range(1, 8):
        knots.append([q0[0], q0[1] - 0.0028 * i, q0[2] - 0.0102 * i, 0.0, q0[4] + 0.0130 * i, q0[5]])
    via = [0.2424, -1.32094, 1.47886, 0.0, 1.41287, 0.2424]
    transit = hermite_fallback(knots + [via])
    t0 = transit[-1]["time_s"]
    weld = [0.2424, -1.30113, 1.54999, 0.0, 1.32194, 0.2424]
    approach = [{"positions": [a + (b - a) * k / 10 for a, b in zip(via, weld)], "time_s": round(t0 + 0.1 * k, 4),
                 "phase": "approach"} for k in range(1, 11)]
    return transit + approach


class RetimeTest(unittest.TestCase):
    def ratio(self, traj):
        return U.acc_ratio([p["time_s"] for p in traj], [p["positions"] for p in traj], A_MAX)[0]

    def test_the_hermite_fallback_overshoots_at_the_liftoff_transit_junction(self):
        traj = liftoff_then_transit()
        r, t, j = U.acc_ratio([p["time_s"] for p in traj], [p["positions"] for p in traj], A_MAX)
        self.assertGreater(r, 3.0)            # as on the server: 4.76 x a_max
        self.assertEqual(j, 0)                # a1
        # the overshoot: a1 swings back although it is constant before and rises after the junction
        a1 = [p["positions"][0] for p in traj]
        self.assertLess(min(a1), a1[0] - 0.01)

    def test_repair_brings_the_leg_under_a_max_and_keeps_its_ends_and_the_approach(self):
        traj = liftoff_then_transit()
        new, rep = U.repair_leg(traj, A_MAX, V_MAX)
        self.assertTrue(rep["changed"], rep)
        self.assertLessEqual(self.ratio(new), U.TARGET)
        self.assertLessEqual(rep["acc_after"], U.TARGET)
        self.assertEqual(new[0], traj[0])
        self.assertEqual(new[-1], traj[-1])
        self.assertEqual([p for p in new if p["phase"] == "approach"], [p for p in traj if p["phase"] == "approach"])
        times = [p["time_s"] for p in new]
        self.assertEqual(times, sorted(times))
        self.assertEqual(len(set(times)), len(times))
        # no reversal: a1 never goes below where it starts
        self.assertGreaterEqual(min(p["positions"][0] for p in new), traj[0]["positions"][0] - 1e-6)
        self.assertLessEqual(U.seg_vel_ratio(times, [p["positions"] for p in new], V_MAX), 1.0)
        w = rep["windows"][0]
        self.assertLess(w["t_b"] - w["t_a"], 1.0)

    def test_a_velocity_step_is_blended(self):
        # a joint at rest that starts at 0.4 rad/s at t = 1 s: 0.4 / 0.05 = 8 rad/s^2 on btexport's metric
        traj = [{"positions": [0.0] * 6, "time_s": round(0.1 * k, 4)} for k in range(11)]
        traj += [{"positions": [0.04 * k] + [0.0] * 5, "time_s": round(1.0 + 0.1 * k, 4)} for k in range(1, 21)]
        traj.append({"positions": [0.8] + [0.0] * 5, "time_s": 3.5})
        self.assertGreater(self.ratio(traj), 2.0)
        new, rep = U.repair_leg(traj, A_MAX, V_MAX)
        self.assertTrue(rep["changed"])
        self.assertLessEqual(self.ratio(new), U.TARGET)
        self.assertEqual((new[0], new[-1]), (traj[0], traj[-1]))

    def test_a_smooth_leg_is_left_alone(self):
        traj = [{"positions": [0.5 * (1 - __import__("math").cos(3.14159265 * k / 40))] + [0.0] * 5,
                 "time_s": round(0.1 * k, 4)} for k in range(41)]
        new, rep = U.repair_leg(traj, A_MAX, V_MAX)
        self.assertIs(new, traj)
        self.assertFalse(rep["changed"])

    def test_plan_in_place_and_robot_limits(self):
        traj = liftoff_then_transit()
        plan = {"behaviour_tree": {
            "root": {"type": "Sequence", "children": [
                {"type": "ExecuteTrajectory", "ports": {"robot_ns": "robot1", "trajectory": "{traj_robot1_W04}"}},
                {"type": "MoveHome", "ports": {"robot_ns": "robot2", "trajectory": "{home}"}}]},
            "blackboard": {"traj_robot1_W04": traj, "home": traj[:3], "other": "x"}}}
        with tempfile.TemporaryDirectory() as d:
            y = Path(d) / "robots.yaml"
            y.write_text("robots:\n  arms:   # ns\n    robot1:\n      name: kr\n"
                         "      joint_velocity_limits:     [2.7, 2.7, 2.7, 5.7, 5.7, 10.7]\n"
                         "      joint_acceleration_limits: [3, 3, 3, 6, 6, 8]   # REVIEW (rad/s^2)\n"
                         "      tool:\n        x: 1\n    robot2:\n      joint_acceleration_limits: [6, 6, 6, 6, 6, 8]\n"
                         "  max_velocity_scaling: 0.7\n")
            lim = U.robot_limits(y)
        self.assertEqual(lim["robot1"]["a_max"], A_MAX)
        self.assertEqual(lim["robot2"], {"a_max": [6, 6, 6, 6, 6, 8], "v_max": None})
        self.assertNotIn("tool", lim)
        info = U.retime_plan(plan, lim)
        self.assertTrue(info["changed"])
        self.assertEqual(info["legs"]["traj_robot1_W04"]["robot"], "robot1")
        self.assertLessEqual(self.ratio(plan["behaviour_tree"]["blackboard"]["traj_robot1_W04"]), U.TARGET)
        self.assertEqual(plan["behaviour_tree"]["blackboard"]["other"], "x")
        self.assertTrue(U.summary_lines(info))
        json.dumps(plan)
        self.assertFalse(U.retime_plan({"tree_id": "bt_1", "status": "ok"})["changed"])   # no tree: nothing to do


if __name__ == "__main__":
    unittest.main()
