"""Robot-controller feasibility of exported plans, and a smoothing post-processor (docs/p3_ctrl.md, track K).

Modules
  limits       controller profiles (velocity / acceleration / jerk fractions, interpolation cycles)
  plans        canonical plans of every controller -> :class:`plans.Plan` (per-robot dispatched legs, holds)
  drivers      how a controller turns dispatched waypoints into per-cycle commands (umarell sim controller,
               umarell's ros2_control driver = cubic segments with zero velocity at every waypoint, JTC variants)
  servo        simulated joint tracking response (cascaded P / velocity loop, saturation, latency)
  feasibility  waypoint / segment checks, per-cycle dynamics, stop-and-go metrics, tracking summary
  smooth       C2, jerk-limited smoothing (quintic P-splines per arm between rests) + common-clock time scaling
  safety       dense re-check: umarell sphere gate, floor / box rules, hulls (self, arm-arm, floor, fixture),
               rule W, joint limits, weld poses / entry / still holds (umarell cells); trajgen's verifier (scenes)
  export       smoothed plan -> umarell BT plan (positions + velocities + accelerations on the cycle grid)

Pure numpy / scipy, plus env/ (MuJoCo) and trajgen/ for the safety re-check. Nothing here changes a shared module.
"""
