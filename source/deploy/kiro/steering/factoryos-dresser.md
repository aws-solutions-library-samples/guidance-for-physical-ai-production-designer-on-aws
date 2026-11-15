# Physical AI Production Designer: dressing robots and the tool's orientation (dresser_*)

* dresser_state (after dresser_open) lists robots (urdf-robot-collection), tools (urdf-tool-collection) and dressed
  robots (urdf-dressed-robots/<robot>__<tool>). A tool without a TCP (W030) gets an estimated one: check dresser_get_tcp /
  dresser_tool_tip and set it with dresser_set_tcp (the user confirms the numbers).
* dresser_dress merges the tool onto the flange: orient in 90° steps about X/Y/Z ([0,0,0] = auto-orient), then
  spin_deg about the flange axis (any angle). It runs in the background: dresser_state until busy is empty, then
  read `dress` (output_rel, diagnostics, error).
* Which orientation makes sense: dresser_orient_sweep in a scene with the product placed and the robot(s) in it
  (scene_file or cell). Start with the default candidates (auto-orient spun 0/90/180/270°), mode quick; add spins
  (e.g. 45°, 135°) or other orient steps when two are close. Report the ranking (welds reached, reached twice, the
  unreached ones and why) and recommend one; apply (writes the dressed robot) only after the user agrees, then
  scene_move_robot dressed=<robot>__<tool> if the scene used another dressing.
* robot_describe dressed=<robot>__<tool>: what a dressed robot is, from its URDF (joints, reach dome, tool length,
  placement figures); use its numbers, never a model's remembered ones.
* estimate_only runs in seconds but is blind to the tool's orientation (domes only): use it only to compare robots.
