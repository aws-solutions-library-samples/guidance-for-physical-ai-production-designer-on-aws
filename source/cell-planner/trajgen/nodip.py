"""No-dip rule (round 3, user decision 7 of results/round3/DECISIONS.md; docs/r3_nodip.md): the electrode / tool must
never enter a cube / workpiece box, at any distance from a weld, except the weld contact itself.

Rule ``nodip-v1`` (``NODIP_VERSION`` 1). When it is on for a scene with workpiece boxes:

* **Tool and arm hulls.** Every URDF collision hull except the electrode hull(s) vs every workpiece box: signed
  distance >= ``obstacles.box_hull.min_distance_m``. The rule forces ``obstacles.box_hull.workpiece: true``.
* **Electrode hull(s).** ``tool.electrode_hulls`` (default: the tip-link hull containing the TCP) vs every workpiece
  box, at every sample:
  - planners (trajgen's IK strokes and transits, which the env's primitives use too): signed distance >=
    ``PLAN_MIN_DISTANCE_M`` (1 mm);
  - verifier / scorer, and the env's check of the executed motion (envn.geom.Checker): signed distance >= 0, i.e.
    any penetration fails (a near miss below the planning margin is not a failure).
* **Contact exemption, the only exception.** Robot r's electrode may intersect a workpiece box only inside the
  contact of a weld w that r is welding. Each MuJoCo contact (electrode hull, box) is exempt iff all three hold:
  1. *time*: the sample lies in r's contact window on w. The window is the dwell of r's weld event
     [t_arrive, t_end_dwell], extended over the contiguous samples just before and after it in which r's TCP stays
     inside the contact ball. These samples are the last millimetres of the descent and the first of the lift-off,
     where contact is made and broken;
  2. *place*: the TCP is within ``CONTACT_TCP_RADIUS_M`` (5 mm) of the weld point p_w (the ball around the weld TCP),
     and the contact point (MuJoCo's contact position) is within ``CONTACT_POINT_RADIUS_M`` (10 mm) of p_w;
  3. *depth*: the penetration is <= ``CONTACT_DEPTH_M`` (1.5 mm). The weld poses themselves measure <= 0.84 mm: the
     tilted 2 mm tip disc on the cube top, 666 v2 test + OOD weld options.
  Any other contact in the same sample (for example the electrode's cone side against a taller neighbouring cube)
  is not exempt.
* **Planners and env** have no clock, so they use the static form of the exemption: conditions 2 and 3 for any weld
  of the task, or for the welds a call names (``SceneModel.electrode_margin(..., welds=...)``). Transits pass
  ``welds=()`` and get no exemption.
* **Checks.** The verifier runs the time-aware form on a grid of at least ``SCORE_HZ`` (400 Hz), together with the
  box-hull check. The continuous certificate (trajgen.certify) runs the static form with the scorer threshold.

The rule replaces round 2's electrode off-weld rule (a 50 mm exemption ball around every weld and a 5 mm scorer
tolerance), and with it the unchecked 20-50 mm near-weld band (docs/n2_eval.md §8.3). It is carried as
``trajgen.scene.ElectrodeRule(nodip=True, ...)``. The verifier's check keeps its name ``electrode_off_weld``, so the
contract tuple ``SUCCESS_CHECKS`` is unchanged; ``electrode_off_weld.rule`` says ``nodip-v1``.

Switch:
* scene key ``obstacles.box_hull.no_dip``: ``true`` / ``false`` / a mapping {enabled, version, contact_tcp_radius_m,
  contact_point_radius_m, contact_depth_m, min_distance_m};
* absent key: ``default_on()``, which is the environment variable ``TRAJGEN_NODIP`` (``1`` on, ``0`` off) if set,
  else ``DEFAULT_ON``;
* ``trajgen.scene.with_nodip(scene, on)`` and ``python -m trajgen --nodip on|off|scene``.
Scenes without workpiece boxes are unaffected: the electrode can only touch workpieces, and obstacle boxes already
check every hull, the electrode included.
"""
from __future__ import annotations

import os

NODIP_VERSION = 1
RULE_ID = f"nodip-v{NODIP_VERSION}"

CONTACT_TCP_RADIUS_M = 0.005      # TCP within this of the weld point (the ball around the weld TCP)
CONTACT_POINT_RADIUS_M = 0.010    # MuJoCo contact position within this of the weld point
CONTACT_DEPTH_M = 0.0015          # max electrode penetration inside the exemption
PLAN_MIN_DISTANCE_M = 0.001       # planners / env: electrode - workpiece signed distance >= this outside it
SCORE_TOL_M = 0.0                 # verifier: fails at a penetration > this outside the exemption
SCORE_HZ = 400.0                  # minimum grid of the verifier's box-hull and electrode checks

DEFAULT_ON = True                 # absent scene key: on (round-3 default since NODIP_READY, 2026-10-17)
ENV_VAR = "TRAJGEN_NODIP"


def default_on() -> bool:
    """The rule's state for scenes that do not declare ``obstacles.box_hull.no_dip``: ``$TRAJGEN_NODIP`` (0 / 1,
    off / on, false / true) when set, else ``DEFAULT_ON``."""
    v = os.environ.get(ENV_VAR)
    if v is None or v.strip() == "":
        return bool(DEFAULT_ON)
    return v.strip().lower() not in ("0", "off", "false", "no")


def rule_dict() -> dict:
    """The rule's constants (NODIP_READY.json, reports, cache keys)."""
    return {"rule": RULE_ID, "version": NODIP_VERSION, "contact_tcp_radius_m": CONTACT_TCP_RADIUS_M,
            "contact_point_radius_m": CONTACT_POINT_RADIUS_M, "contact_depth_m": CONTACT_DEPTH_M,
            "plan_min_distance_m": PLAN_MIN_DISTANCE_M, "score_tol_m": SCORE_TOL_M, "score_hz": SCORE_HZ,
            "default_on": bool(DEFAULT_ON), "env_var": ENV_VAR}
