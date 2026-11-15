# Physical AI Production Designer: running the behaviour tree and the 3D view (bt_*, view_camera)

* A plan (cell_plans) is a behaviour tree; bt_load_plan compiles it into a timeline in the simulated cell (it starts
  the cell-runner if needed) and answers the timeline (operations per robot, t0 / t_end), warnings and collisions.
  Mention a collision finding before playing.
* Playback: bt_play, bt_pause, bt_stop (rewinds), bt_seek t, bt_seek_operation op_id, bt_step (next / back operation;
  robot_id for one robot), bt_set_speed, bt_set_loop, bt_set_breakpoints operations [...] (playback pauses when one
  starts; bt_state break_hit says which), bt_unload.
* bt_state: playing, t / duration, each robot's phase (motion / hold / idle) and operation, breakpoints.
* Walking the user through a plan: breakpoints on the welds of interest, bt_play, at each hit view_camera action focus
  weld <id> (or robot <id>), explain, bt_play again.
* view_camera: set (eye, target), focus (target point, robot or weld; distance), preset (iso, top, front, back, left,
  right), orbit (yaw_deg, pitch_deg), zoom (factor), fit (the whole scene). page: dresser, fitter, product, training,
  live (default: the page shown). It moves the view even if the user moved it: say so when you do.
