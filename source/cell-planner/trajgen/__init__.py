"""trajgen: scene/task loader and hand-written (scripted, non-learned) weld trajectory generation for 1..4 robots.

    from trajgen import generate
    res = generate("scenes/welding_cell_2_small/scene.yaml", "scenes/welding_cell_2_small/task.json", out="data/trajgen/x")

CLI: ``python -m trajgen --scene ... --task ... --out ...`` (docs/trajgen.md).

Modules:
    trajgen.scene        scene.yaml + task.json loader and validator (scenes/SCHEMA.md, schema_version 2)
    trajgen.robot        RobotModel: one robot's URDF chain at its base pose (numpy FK, spheres, hull vertices)
    trajgen.ik           SceneIK: expert.ik.WeldIK's solver on per-robot Pinocchio models built from the scene
    trajgen.mjscene      MjWorld: MuJoCo hull models for any robot count (self, world/boxes, robot pairs)
    trajgen.tool_spheres tool collision spheres fitted to the URDF hulls (origin rotations applied)
    trajgen.checks       scene collision geometry (link + tool spheres, floor rules, arm-arm clearance)
    trajgen.model        SceneModel: every rule of the scene in one object (spheres, boxes, rule W, MuJoCo hulls)
    trajgen.reach        reachability + 5-DOF weld IK with descent / lift-off strokes (with failure reasons)
    trajgen.alloc        allocation and per-robot order (two-robot sweep; N-robot greedy + local search)
    trajgen.motion       S-curve time parameterisation, segments, joint-space transit planning (vias, RRT-Connect)
    trajgen.schedule     speeds, legs and the dense pair check
    trajgen.search       two-robot step search (umarell's sync-at-rest model)
    trajgen.coord        N-robot prioritized planning on one clock (waits, yields, deadlock-free)
    trajgen.ntimeline    N-robot timeline (player.timeline-compatible file format)
    trajgen.btplan       umarell BT plan JSON for any robot count + its executor replay
    trajgen.verify       dense verification of any N-robot timeline against a scene
    trajgen.synth        synthetic 1/3/4-robot test scenes
    trajgen.pipeline     the pipeline (generate()) and its outputs
"""


def generate(scene_path, task_path=None, **opts):
    """See trajgen.pipeline.generate."""
    from trajgen.pipeline import generate as _g

    return _g(scene_path, task_path, **opts)
