# Driving the Demo Cell from ROS 2 (Windows + Pixi + Isaac Sim)

This guide walks through testing the per-robot ActionGraphs that
`robo-fitting-room` emits — manually moving joints with `ros2 topic pub`,
reading joint state, and confirming collisions stop the arm.

The examples use a manifest named `Demo Cell` with a six-axis robot `R1`
whose URDF joints are `joint_1` … `joint_6`; substitute your cell's sanitised
name, your robot IDs and your URDF's joint names (see
[the topic schema](concepts.md#topic-schema)).

Assumes:

- Isaac Sim 5.1 or later installed at `C:\isaac-sim`
- The composed scene (`demo-cell.zip`) extracted and its `.usda` opened
- A Pixi environment on your Windows box that already includes ROS 2 Humble
  (or Jazzy) — referred to below as the **ROS terminal**
- An empty PowerShell where Isaac Sim is launched — the **Isaac terminal**

If `ros2 topic list` already works in your Pixi env you can skip directly to
[§3 — Verify topics](#3-verify-topics).

---

## 1. Pre-flight inside Isaac

1. **Window → Extensions** → search **`isaacsim.ros2.bridge`**.
2. Toggle **Enabled** (green dot). If the toggle says "library not found",
   set the DDS environment before launch (next step).
3. Quit Isaac.
4. Edit (or create) the launcher batch file you use to start Isaac and set the
   ROS 2 environment **before** the kit executable line. Example
   `isaac-sim.bat`:

   ```bat
   @echo off
   set RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   set ROS_DOMAIN_ID=0
   rem Only if you use a FastDDS profile: its full path (cmd expands no wildcards here)
   rem set FASTRTPS_DEFAULT_PROFILES_FILE=C:\path\to\fastdds.xml
   call "C:\isaac-sim\isaac-sim.bat"
   ```

   Setting `ROS_DOMAIN_ID=0` matters: the ROS terminal must use the **same**
   domain or it won't see Isaac's topics.

5. Relaunch Isaac via that script. During startup the console should report the
   `isaacsim.ros2.bridge` extension starting without errors.

## 2. Pre-flight in the ROS terminal (Pixi)

```powershell
# In a fresh shell — NOT inside Isaac:
cd C:\path\to\your\pixi\workspace
pixi shell
$env:ROS_DOMAIN_ID = "0"      # PowerShell; in cmd: set ROS_DOMAIN_ID=0
ros2 doctor --report
```

If `ros2` is not on PATH inside the Pixi shell, your env is missing the
`ros-humble-desktop` (or `ros-jazzy-desktop`) feature. Add it to
`pixi.toml`, run `pixi install`, then re-enter the shell.

## 3. Verify topics

Open the composed scene, press **Play** (spacebar), then in the ROS
terminal:

```powershell
ros2 topic list
```

Expected — per robot a joint-state pair, plus the cell's clock (and ROS 2's own
`/parameter_events` and `/rosout`):

```
/Demo_Cell/R1/joint_command
/Demo_Cell/R1/joint_states
/Demo_Cell/R2/joint_command
/Demo_Cell/R2/joint_states
/Demo_Cell/clock
```

Every topic sits under the cell: `/<cell>/<robot>/…`, both segments sanitised
(`Demo Cell` → `Demo_Cell`; characters outside `[A-Za-z0-9_]` become `_`, a
digit-leading ID gets an `r_` prefix: robot `42` → `/Demo_Cell/r_42/…`). The
per-link contact topics, `/Demo_Cell/R1/contacts/<link>` with the link's
[prim name](concepts.md#topic-schema) (`link-6` → `link_6`), publish only when
the link touches something, so they may not be listed before a first contact.

## 4. Read live joint state

```powershell
ros2 topic echo /Demo_Cell/R1/joint_states --once
```

You should see a `sensor_msgs/JointState` message with one name per actuated
joint (`joint_1` … `joint_6`, the URDF's own names) and matching `position`,
`velocity`, `effort` arrays. Positions are in **radians** (metres on a prismatic
joint) — the bridge respects SI even though the stage's joint attributes are in
degrees. At Play they are the robot's home pose: the manifest's `home_pose_deg`,
or the stowed default `[0°, −90°, +90°, 0°, 0°, 0°]` = `[0, -1.5708, 1.5708,
0, 0, 0]` rad.

If nothing comes back, jump to [§7 — Troubleshooting](#7-troubleshooting).

## 5. Send a joint command

```powershell
ros2 topic pub --once /Demo_Cell/R1/joint_command sensor_msgs/msg/JointState `
  "{name: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'], `
    position: [0.5, -1.5708, 1.5708, 0.0, 0.7, 0.0]}"
```

The arm should track to that pose: from the default home this turns
`joint_1` and `joint_5` while keeping the home elbow.

**Important:**

- Positions are **radians** for revolute joints, **metres** for prismatic.
- The `name` array must match the **URDF joint names exactly**: the robot's
  actuated joints, as `joint_states` lists them. Fixed joints (e.g. a tool
  flange's, or `world_anchor`) are neither published nor commanded.
- Sending `velocity` or `effort` mid-stream conflicts with the position
  drive — drop them unless you specifically know you want torque control.

## 6. Drive in a loop (collision test)

This loop swings `joint_1` back and forth while keeping the elbow folded; aim
the swing so the arm physically meets a fixture. The robot links carry their
mass model's masses and convex-hull collisions, so the drive will **stall**
instead of penetrating the fixture, and the joint position will plateau before
the commanded value. That's how you confirm collision response is working.

```powershell
$names = "['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']"
foreach ($i in 1..5) {
  foreach ($a1 in "-1.5", "1.5") {
    ros2 topic pub --once /Demo_Cell/R1/joint_command sensor_msgs/msg/JointState `
      "{name: $names, position: [$a1, -1.5708, 1.5708, 0.0, 0.0, 0.0]}"
    Start-Sleep -Seconds 4
  }
}
```

Run that, watch the viewport (and, in a second ROS terminal,
`ros2 topic echo /Demo_Cell/R1/joint_states`): `joint_1` should swing freely
until the arm meets a fixture or another robot, then its position should hold
short of the commanded angle even though the command repeats. Meanwhile the
touching link's `/Demo_Cell/R1/contacts/<link>` topic should publish
`data: true`.

## 7. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `ros2 topic list` empty during Play | Bridge ext disabled, or `ROS_DOMAIN_ID` mismatch | Re-check §1 step 4; both terminals must show the same `ROS_DOMAIN_ID` (in PowerShell `$env:ROS_DOMAIN_ID`; `set` there doesn't set an environment variable) |
| Topics visible but `echo` blank | OmniGraph isn't ticking | In Isaac, **Window → Visual Scripting → Action Graph**, open `/World/ActionGraph_R1`, watch the green pulse on the `OnPlaybackTick → PublishJointState` link |
| `pub` succeeds but robot ignores it | `name[]` mismatch, or the wrong topic | Verify each name with `ros2 topic echo /Demo_Cell/R1/joint_states` first; names must match the URDF case-for-case, and the topic is under the cell's namespace |
| Robot twitches/snaps to target | The drives are stiff by design: 1.5e5 N·m/deg (8.6e6 N·m/rad), damped at ζ 0.7 on the inertia each joint moves, so a large jump in the command is followed as fast as the drive's force cap and velocity limit allow | Command smaller steps or a trajectory. The gains are constants in `src/drives.rs` (`DRIVE_STIFFNESS`, `DRIVE_DAMPING_RATIO`, `LINEAR_DRIVE_SAG_M`); the masses they're tuned on follow the manifest's `link_density_kg_m3` and `payload` — recompose after changing either |
| `joint_states` missing some joints | Fixed joints aren't published, that's expected | The actuated joints of each arm are what the bridge reports |
| Two robots see each other's commands | Both cells have the same sanitised `name` and robot IDs on one domain | Give the manifests distinct `name`s (topics are `/<cell>/<robot>/…`), or use distinct `ROS_DOMAIN_ID`s |
| Wall-clock vs sim-clock mismatch in MoveIt/rviz | Subscriber not using sim time, or listening on `/clock` | The clock is cell-scoped: `ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true -r /clock:=/Demo_Cell/clock` |

## 8. Useful one-liners

```powershell
# What's the rate of joint_states?
ros2 topic hz /Demo_Cell/R1/joint_states

# Discover the message type the publisher is using
ros2 topic info /Demo_Cell/R1/joint_states --verbose

# Plot a joint's position over time (rqt_plot must be in your Pixi env)
ros2 run rqt_plot rqt_plot /Demo_Cell/R1/joint_states/position[0]

# Echo the simulation clock — confirms the scene-level ActionGraph is alive
ros2 topic echo /Demo_Cell/clock --once
```

---

When step 4 (`echo joint_states`) returns data but step 5 doesn't move
the robot, paste the Isaac console output (filter on `ros2.bridge` and
`omni.graph`) and we'll narrow it down.
