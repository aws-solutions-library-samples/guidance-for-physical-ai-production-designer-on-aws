---
name: robot-cell-layout
description: How to place industrial robots (any serial arm with a tool, e.g. a spot welding gun) around a part so they reach its welds comfortably, not just barely. Discover the robot's own figures with robot_describe, then reason about reach ratio, approach direction, working height, spacing, robots per side and pedestals, and measure and fix a layout with scene_auto_place_robots, scene_layout_quality and scene_reach_check. Use it whenever the user asks to place, move, add or remove robots, or asks why robot motions look awkward.
---

# Placing robots around a part (Physical AI Production Designer)

"Reachable" is not "good". A robot folded up close to its weld, stretched to its limit, or twisting its wrist
far round to point the gun reaches the weld, but moves awkwardly and slowly, near joint limits and wrist
singularities, and the planner struggles. Place for comfort, and measure it; never place by eye.

Never use remembered numbers for a robot model. Every robot is different (reach, shoulder height, base size,
tool length): take its numbers from **robot_describe** and reason from them.

## 1. What makes a placement comfortable

All measured at the wrist centre (A5's origin: the tool length behind the weld, against the approach). The
ratios and angles are generic: they are shares of the robot's own reach.

| Measure | Comfortable | Why |
|---|---|---|
| Reach ratio: wrist distance from A1 / the robot's reach at that height | 0.45-0.80, best ~0.62 | Below ~0.4 the arm folds up (A3 near its limit, the wrist near A1: a singularity zone; the inner dome is dead space). Above ~0.85 it is stretched: slow, little margin for the approach stroke and for path deviations. |
| Approach alignment: angle between the gun's approach (horizontally) and the robot's line out to the wrist | ≤ 50° | The robot pushes the gun outward along its own reach. A large angle means A4/A5/A6 turn far round for every weld, close to wrist singularity (A5 ≈ 0), and swing between welds. |
| Height: the wrist's height | inside the robot's comfortable height band (`dome.comfortable_height_band_m`: where its reach is ≥ 90 % of its farthest) | Top or bottom of the dome: A2/A3 near limits. Fix with a pedestal (raises the robot) or the product's height. |
| Spread about A1 of the welds one robot does | ≤ ~120° | Every weld change is a long A1 swing; split the welds between robots. |
| Base spacing | ≥ the robot's `placement.min_spacing_m` | A neighbour's base stays out of the arm's own folded zone, and the bodies don't touch; closer robots constantly wait for each other in interlock zones. |
| Clearance from the part | ≥ the robot's `placement.clearance_m` | Its base footprint (base and the turning column) plus a margin; add the gun's back swing and cable dress where the hall needs it. |

## 2. Know the robot: robot_describe

Call **robot_describe** first, for the scene's dressed robot(s) (`scene_file` or `cell`, or `dressed=<robot>__<tool>`
for one not placed yet). It answers, from the URDF:

* `dof`, `joints` (limits in degrees, max velocity), `a1_height_m`, `shoulder_height_m` and `shoulder_offset_m`
  (A2 against A1), `links` (segment lengths at the zero pose).
* `dome`: `r_max_m` (the wrist's farthest reach), `sweet_height_m` (the height it reaches farthest at),
  `comfortable_height_band_m`, `r_min_at_sweet_m` (closer than this the wrist can't fold).
* `tool`: `tool_length_m` (wrist to TCP), `tcp_offset_m` in the flange frame, `tcp_declared` (false: an estimate;
  say so, the figures are approximate).
* `placement` (with its formulas in `placement.rules`): `comfortable_wrist_radius_m`, `weld_distance_m` and
  `comfortable_weld_distance_m`, `weld_height_band_horizontal_m` / `weld_height_band_top_down_m`,
  `weld_line_span_m`, `footprint_radius_m`, `clearance_m`, `min_spacing_m`.

All heights are above the robot's mounting face: on a pedestal, add `pedestal_mm / 1000`.

Tell the user the few figures that drive the layout (reach, sweet height, tool length, weld distance) before placing.

## 3. From the figures to a placement

* **Group welds by the side they are approached from** (the approach vectors): one group per side of the part
  (left/right of a car body, top, …). A robot works one side; never ask a robot to reach round the part.
* **Distance from the weld plane**: for welds approached horizontally, put the base (A1 axis) `weld_distance_m`
  behind the weld plane, along the approach (the gun pushed outward along the robot's reach); anywhere in
  `comfortable_weld_distance_m` is comfortable. Standing much closer folds the arm up (cramped), much farther
  stretches it. For other approaches use the wrist: it must be `comfortable_wrist_radius_m` from A1, and the wrist
  is `tool_length_m` behind the weld against the approach.
* **Height → pedestal**: compare the welds' heights with `weld_height_band_horizontal_m` (level tool) or
  `weld_height_band_top_down_m` (welds from above). Welds above the band: a pedestal of about (weld height − band
  centre), rounded to 100 mm; below it: a lower product stand, or a higher product support. Prefer welds near
  `sweet_height_m`.
* **Robots per side**: compare the weld span along a side with `weld_line_span_m` (what one robot at the weld
  distance does comfortably, an upper bound): robots per side ≈ span / weld_line_span_m, rounded up, and more when
  one robot's welds would spread over ~120° about A1. A single robot covering more has to twist for the far welds
  (the measure says "twisted"); tell the user this trade-off when they ask for fewer robots.
* **Spacing and clearance**: bases at least `min_spacing_m` apart and `clearance_m` from the part's box: the
  defaults scene_auto_place_robots uses. If robots per side don't fit at that spacing along the part, use fewer
  per side, or stagger them (one nearer, one farther, both inside the comfortable distance).
* **Symmetric parts get symmetric layouts** (mirror positions, yaw facing the part).
* **Yaw**: A1 facing the centre of the robot's own welds (the tools set it).
* **Cycle time**: split the welds evenly between robots; heavily unequal shares make one robot the bottleneck.

## 4. The tools, in order

1. **scene_current**: the open scene: robots, products, how the welds are reached now.
2. **robot_describe** for its dressed robot(s) (section 2).
3. **scene_layout_quality** on the current scene: per weld its best robot and fit (comfortable, cramped,
   stretched, twisted, too_high, too_low, unreachable), per robot its welds by fit and advice with a suggested
   base, and notes on bases under their robot's spacing or clearance. Report this before changing anything (what
   is wrong, in plain words, against the robot's figures).
4. Ask what to optimise, with your recommendation (from section 3: how many robots per side the weld span needs):
   the same number of robots at better places (min_robots = max_robots = N), or the fewest that do every weld
   comfortably (min 1, max ~6); coverage 2 only when the user wants redundancy.
5. **scene_auto_place_robots** (dressed = the scene's robot, min/max as chosen; leave min_spacing_m / clearance_m
   out to use the robot's own): present every layout returned (robots, bases [x, y, yaw], comfortable welds of all,
   per robot fits and advice) and recommend one; check the bases against your estimate from the figures (distance
   from the weld plane ≈ weld_distance_m) and say plainly when a layout with the asked-for count leaves welds
   twisted and how many robots would make them comfortable. Its notes say where spacing and clearance came from.
6. With the user's go, write it (**write_as** `<scene>-<n>r`, a new scene name; never overwrite), **scene_open** it,
   **view_camera** fit.
7. **scene_layout_quality** on the written scene, then **scene_reach_check** mode full (the planner's IK with the
   approach stroke). Fix what is left with **scene_move_robot**: use the suggested_base the quality tool gives
   (small steps, then rate again), pedestal_mm for height, or another tool orientation (dresser_orient_sweep).
8. Placing by hand (the user names positions): always rate it with scene_layout_quality afterwards and say what
   is cramped or twisted and the suggested base, before the user plans with it.

Never stop at "every weld is reachable": report the comfort, and do not present a layout with cramped or
stretched welds as good.

## 5. Tuning

`comfort` on both tools: `ratio_min` / `ratio_target` / `ratio_max` (default 0.45 / 0.62 / 0.80),
`align_max_deg` (50); `height_band_m` only to override the robot's own height band (± that many metres about its
sweet height). Tighter for fast cycle times (ratio 0.5-0.75, align 40), looser for a cramped hall (ratio
0.4-0.85). `min_spacing_m`, `clearance_m` on scene_auto_place_robots only for the hall's rules (safety fences,
cable ducts, maintenance access); by default they are the robot's own.

The measures come from the reach dome (no IK, milliseconds); the IK check (scene_reach_check full) confirms the
approach strokes and joint limits. Both together before planning.
