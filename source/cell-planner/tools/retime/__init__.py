"""Speed-profile retimer for any controller's timeline (track SPEED, docs/p3_speed.md).

    python -m tools.retime --timeline X.npz [--cell ID | --scene DIR] --profile full|umarell --out Y.npz [--bt-out P]

Modules
  context  cell / scene context: URDF joint limits (robotmodel.URDFRobot) + acceleration override, welds, FK,
           arm-arm clearance, dense re-check (tools.ctrl.safety / trajgen.verify)
  topp     time-optimal parameterisation with path-varying v / a / jerk limits, S-curve filter
  core     per-arm paths, weld-operation labels, profiles, shared clock (sync waits / coupled), outputs
  bt       umarell behaviour-tree export (plan JSON + BTCPP v4 XML) that keeps the retimed shared clock
"""
