## Tool URDF Requirements

**Strict requirement:** The tool URDF origin point `(0, 0, 0)` MUST be located at the exact center of the tool's mounting flange interface.

This is the docking point where the tool connects to the robot's end-effector. The dresser uses this origin as the rotation pivot and attachment reference.

### Rules

1. The coordinate origin `(0, 0, 0)` must coincide with the center of the tool's mounting/docking flange
2. All tool geometry (visual and collision meshes) is positioned relative to this docking center
3. The tool's mounting surface faces should be approximately at the origin plane
4. Tool geometry extends away from the origin into the working direction

### Tool centre point (TCP)

Declare the tool's working point (a stick tool's electrode tip, a gun's TCP) as a link named `tcp`
on a fixed joint from the root link, its origin in the docking-interface frame:

```xml
<link name="tcp"/>
<joint name="tcp_joint" type="fixed">
  <origin xyz="0.6695255 -0.0005 -0.0056" rpy="0 0 0"/>
  <parent link="World"/>
  <child link="tcp"/>
</joint>
```

The Robo Dresser carries it into the dressed robot as a fixed `tcp` frame on the flange (mounting
rotation applied), and Prepare Cell gives the planner its offset as `tcp_offset`. A tool without
one dresses with no TCP frame, and the planner's TCP is only estimated from the tool's meshes.

The Robo Dresser's TCP section edits this declaration (xyz in mm, rpy in degrees, relative to the
root link; "Snap to tool tip" puts it at the far end of the collision geometry along the tool
axis). Saving rewrites the `tcp_joint` origin here and the `tcp_joint` of every dressed robot made
with the tool (catalogue and cell copies); the cells using those robots need Prepare Cell again.

### Directory Convention

Each tool lives in its own directory: `<TOOL_NAME>/<TOOL_NAME>.urdf` with meshes in `<TOOL_NAME>/meshes/`.

### Note

URDF files and meshes sourced from customer data are not checked in here.
