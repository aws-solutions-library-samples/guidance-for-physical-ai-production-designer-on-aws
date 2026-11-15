# Physical AI Production Designer: building the scene (scene_*)

* A scene is referred to by its name (letters, digits, _, -, .; it is the folder scenes/<name>/scene.yaml): robots (dressed URDF, position, yaw, pedestal) and fixtures (products,
  supports). Scene frame: metres, z up, the floor at z = 0. Every edit saves it; the cells made from it are rebuilt.
* New cell from scratch:
  1. scene_create file_name <name>.
  2. scene_place_product product <p> (position, yaw). Leave height_m out for the automatic height: with robots in the
     scene it is where they reach the welds best; without, give robot <robot>__<tool> (and pedestal_mm) and the welds go
     to that robot's farthest reach. A raised product stands on a generated support box. Say the height chosen.
  3. scene_auto_place_robots scene_file <name> dressed <robot>__<tool> min_robots/max_robots (coverage 2 for
     redundancy): it answers layouts with 1..N robots, the best one, and the welds it misses. Present the options
     (robots, covered welds) and ask which to use; write it with write_as <name>-<n>r (never overwrites).
  4. scene_reach_check (mode full) on the chosen scene: confirm with the planner's IK; fix with scene_move_robot,
     a pedestal, or another tool orientation (factoryos-dresser.md).
  5. scene_create_cell scene <name>: the cell (active cell; Prepare Cell starts, agentic_status follows it).
* Edits by hand: scene_add_robot, scene_move_robot (position, delta, yaw, face_products, pedestal_mm, dressed),
  scene_remove_robots, scene_remove_fixtures. Keep robot bases ≥ 1.5 m apart and ≥ 0.4 m from the product's box.
* scene_open shows a scene in the Robo Fitter; view_camera page fitter to show what you talk about.

## Placing robots

Load the skill robot-cell-layout first, whenever robots are placed, moved, added or removed: "reachable" is not
"good" (cramped, stretched or twisted robots move awkwardly). The flow: scene_current, robot_describe (the scene's
dressed robot: reach, sweet height, tool length, weld distance, spacing, clearance; never assume a model's
numbers), scene_layout_quality (what is wrong now), ask what to optimise (same N robots at better places, or the fewest that do every weld comfortably),
scene_auto_place_robots, present the layouts with their comfort, write_as a new scene with the user's go,
scene_open, scene_layout_quality and scene_reach_check (mode full) on it, fix with the suggested bases. A hand
placement is always rated with scene_layout_quality before the user plans with it.
