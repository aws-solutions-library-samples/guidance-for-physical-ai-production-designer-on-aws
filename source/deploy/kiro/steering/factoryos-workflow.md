# Physical AI Production Designer: the whole workflow from the chat

You drive Physical AI Production Designer on this workspace through MCP tools. The factoryos server's tools are routes of the workspace's
backend; the factoryos-planner-router plans; plan2cell explains and checks control code. The steps, in order (each
has a steering file of its own):

1. **Product & Task** (product_*): the part and its welds. factoryos-product.md
2. **Robo Dresser** (dresser_*): a robot dressed with the tool; which way to mount the tool. factoryos-dresser.md
3. **Robo Fitter** (scene_*, cell_*): product placed at a height on a support, robots placed to reach the welds, the
   cell made from the scene. factoryos-scene.md, factoryos-reachability.md
4. **Planning** (factoryos-planner-router): interactively, the planner recommended. factoryos-planning.md
5. **Behaviour tree** (bt_*) in the simulated cell, and the **3D view** (view_camera). factoryos-bt-view.md
6. **Control code** (codegen_*, process_*, plan2cell_*): KRL and PLC code; the process definition holds the IOs.
   factoryos-control-code.md

## How to work

* Find out before you change: list and read first (product_list / product_get, dresser_state, scene_list / scene_get,
  cell_plans, vcell_status). Never guess names: copy them from what the tools return.
* Ask when the request leaves a choice open that changes the result (which product / robot / tool / cell, a height,
  how many robots, which planner, which export); offer a sensible default with it.
* Say what you are about to change in one or two lines, then call the tool. kiro asks the user before every tool that
  writes; a declined call means: don't, ask what they want instead.
* Read tools (read-only) and moving the simulated playback or the 3D view need no confirmation.
* Destructive tools (the vcell_* that power, deploy, run production or operate a controller) act on a running virtual
  cell and cost money: only with the user's explicit go for that very action, never as a side effect of another request.
* Long jobs (dressing, orientation sweeps, reach checks, plans, exports, deploys) answer at once or take minutes:
  follow them and report changes, not "still running".
* After a change, check it (product_check_welds, scene_reach_check, plan_status, codegen_build_info,
  vcell_deploy_check) and say what passed and what did not, with the fix.
* Show the user where to look: view_camera to the robot or weld you talk about.
