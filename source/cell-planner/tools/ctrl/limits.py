"""Controller profiles: which limits a plan is held to, and on which interpolation cycle.

All limits are fractions of the per-joint robots.yaml / scene limits (v_max rad/s, a_max rad/s^2); jerk is given in
a_max per second (the unit used throughout the repo: umarell's saved plans peak at ~4.5 a_max/s, PPO v3's motion
filter enforces 5, trajgen plans 4).

* ``hardware``: what the drives can do at all, 1.0 v_max / 1.0 a_max. KUKA does not publish acceleration or jerk
  limits for the KR 240 R2900; robots.yaml's values are umarell's (unreviewed defaults, ERRATA E13 / E20). The jerk
  value (10 a_max/s) is a soft reference, not a fault threshold.
* ``umarell``: umarell's planning rule at max_velocity_scaling 0.7: 0.7 v / 0.49 a (a_max * vscale^2,
  time_parameterization.py:138), jerk 5 a_max/s. Zone rules (approach 0.28 v / 0.078 a within 5 cm of the next weld,
  MoveHome 0.4 v / 0.16 a) are reported separately (``ZONE_RULES``).

Interpolation cycles: KUKA KR C4 interpolation (IPO) cycle 12 ms; RSI / the ros2_control KUKA hardware interfaces
run the position loop at 4 ms (250 Hz). umarell's own controllers: the Pinocchio sim controller ticks at 120 Hz
(PUBLISH_RATE_HZ), the Isaac controller streams at 50 Hz.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass

IPO_CYCLE_S = 0.012          # KR C4 interpolation cycle
RSI_CYCLE_S = 0.004          # RSI / ros2_control KUKA position interface
SIM_TICK_S = 1.0 / 120.0     # umarell controller/pinocchio_robot_controller.py PUBLISH_RATE_HZ default
ISAAC_TICK_S = 1.0 / 50.0    # umarell controller/isaac_sim_controller.py PUBLISH_RATE_HZ default
PLAN_DT_S = 0.05             # 20 Hz: the plans' own sample interval


@dataclass(frozen=True)
class Profile:
    name: str
    v_frac: float
    a_frac: float
    j_frac: float            # jerk limit in a_max per second

    def to_dict(self):
        return asdict(self)


PROFILES = {
    "hardware": Profile("hardware", 1.0, 1.0, 10.0),
    "umarell": Profile("umarell", 0.7, 0.49, 5.0),
}

# umarell zone rules (reported, not part of the hardware check): (v_frac, a_frac)
ZONE_RULES = {"approach": (0.28, 0.078), "home": (0.4, 0.16)}
APPROACH_ZONE_M = 0.05


def profile(p) -> Profile:
    return p if isinstance(p, Profile) else PROFILES[str(p)]
