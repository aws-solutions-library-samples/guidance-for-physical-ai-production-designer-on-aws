btgui — view and step through BehaviorTree.CPP trees and factoryos plans
=========================================================================

btgui.app works on macOS 11 or later, on both Apple Silicon and Intel Macs.

First launch
------------
The app is not notarised by Apple, so macOS blocks a plain double-click the first time.
Right-click btgui.app, choose Open, then click Open again in the dialog. After that it
opens normally. If macOS still refuses, run this in Terminal from this folder:

    xattr -dr com.apple.quarantine btgui.app

Using it
--------
btgui asks for a file when it starts, in the examples/ folder. (Moved elsewhere, or
started before macOS lifted the quarantine, the app runs on its own and the picker opens
in a copy of the examples inside it.) Pick a behaviour tree (.xml) or a plan (.json); if
you cancel, the window has an Open... button. Later, open another with File > Open
(Cmd+O), read the same one again with File > Reload (Cmd+R), or drop a file on the
window. Try these:

  bt_1790934549_welding_cell_2_small.json   two robots welding, 75.30 s
  bt_1791274820_cell_2robot.json            one robot's staging move and the way home
  cell_gate_stall.json                      a gate that never opens: Run to wait halts
                                            at 8.50 s; Release it, or Release all
  v4_cell_with_gates.xml                    a tree of stubs: try Overrides > All
                                            actions manual and force them

  Run / Pause     run in real time
  Step            one tick, at the next instant something changes
  Run to wait     run until the tree finishes or halts
  Run to t =      run up to that instant
  tick slider     scrub back through what happened
  drag, scroll    pan the tree; Cmd + scroll or pinch zooms. Fit shows it whole,
                  Follow keeps the running leaves in view, Compact hides port rows
  click a node    inspect its ports, blackboard values and history; edit ports and values
  right-click     force a node to SUCCESS or FAILURE, stop a motion, break before it
                  starts, or change how a leaf runs
  double-click    collapse or expand a SubTree

A plan's robot leaves run the way the cell-runner runs them: each motion is placed on a
timeline, which the Cell panel at the bottom shows per robot, with any warnings. When the
tree cannot go on without you (a gate that never opens, a leaf waiting for you, a
breakpoint), a banner says where and why, and offers the decisions that let it go on.

In a tree that is not a plan, leaves that only a real robot could run are simulated. By
default they succeed after 1 second. "Overrides > All actions manual" makes them wait
for you to force them.

From Terminal you can also pass options:

    btgui.app/Contents/MacOS/btgui examples/bt_1790934549_welding_cell_2_small.json --run
    btgui.app/Contents/MacOS/btgui examples/v4_cell_with_gates.xml --manual --run
