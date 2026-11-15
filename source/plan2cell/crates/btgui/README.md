# btgui

> Part of the plan2cell workspace: run the commands below from the repository root. The crates
> `bt-core`, `bt-exec`, `bt-cell` and `bt-view` live next to it in `crates/`; `bt2plc` reads plans
> through the same `bt-core`.

A Groot2-style view of a [BehaviorTree.CPP](https://www.behaviortree.dev) tree that you can also
execute, written in Rust with egui. It renders the tree the way Groot does (same layout
algorithm, same monitor colours) and runs it with its own BT.CPP 4 interpreter, so you can watch
which nodes are ticked.

A factoryos plan runs the way the cell-runner runs it: its cell leaves place their motion on a
schedule as the tree ticks, so you see every node's status, where the robots are, and where and
why the tree halts — and you decide for it: force a node, switch a leaf's policy, release a
stalled gate, pause at a breakpoint, or edit a port or blackboard value before a node starts.
In a plain tree, leaves that only exist in a real robot cell are stubs you can force to SUCCESS
or FAILURE, which is how you push a tree past whatever it is waiting for.

```
cargo run -p btgui -- test_data/bt_1790934549_welding_cell_2_small.json
```

![the welding plan halted at W09, a weld switched to "wait": Follow has zoomed onto it, the
Cell panel shows the motion placed so far, and the banner offers Succeed, Fail or
Simulate](../../docs/btgui/screenshot.png)

*The welding plan with W09 set to wait for an override (right-click ▸ Policy: wait for
override, or *runs as: wait* in the inspector), after Run to wait: the run halts at 50.90 s.*

## Running it

```
cargo run -p btgui -- [tree.xml | plan.json] [--manual] [--run | --ticks N] [--select NAME]
```

* `--manual` — every action, the cell leaves included, stays RUNNING until you force it.
* `--run` — run in virtual time until the tree finishes or halts.
* `--ticks N` — take N steps, then stop.
* `--select NAME` — select the node with this instance name.

Without a file, btgui opens a file picker as soon as its window is up: in `test_data/` (in the
macOS zip, the `examples/` folder next to the app, or the copy inside the app when it runs on its
own, moved or translocated by Gatekeeper), and later in the folder of the last file opened.
Cancel it and the empty window offers **Open…** instead. Files can also be opened with *File ▸
Open…* (⌘O), read again with *File ▸ Reload* (⌘R), or dropped on the window. (winit does not
hand btgui the file of macOS's "Open With", so that starts with the picker too.)

Besides BehaviorTree XML it reads a factoryos plan in every layout the cell-runner accepts: the
planner's `{behaviour_tree: {xml, blackboard}}` as written into
`<cell>/agentic_planner/plans/bt_*.json`, its JSON mirror of the tree (`root`) when there is no
`xml`, and a bare `.xml` whose blackboard is in the `.json` of the same name next to it. If that
sibling cannot be read or parsed, the error says so and the bare tree loads without it.

```
cargo run -p btgui -- test_data/bt_1790934549_welding_cell_2_small.json --run   # ends at 75.30 s
cargo run -p btgui -- test_data/bt_1791274820_cell_2robot.json --ticks 3        # Step: 0, 0, 4.73 s
cargo run -p btgui -- test_data/v4_cell_with_gates.xml --manual                 # stubs you force
```

### In the window

| | |
|---|---|
| Run / Pause | run in real time, ticking whenever something changes (a timer, a motion ending, a gate that can open) |
| Step | one tick, at the next instant something changes |
| Run to wait | run in virtual time (timers jump) until the tree finishes or halts |
| Run to t = | run in virtual time up to that instant; manual leaves keep waiting meanwhile |
| tick slider | scrub the history; the tree, the robots and the motion are drawn as they were then |
| drag or scroll | pan; ⌘/Ctrl + scroll (or pinch) zooms |
| Fit / Follow / Compact | frame the whole tree; keep the running leaves in view, zoomed in to read them when the tree is too big to read whole (on by default; a stop brings its node into view the same way); draw the cards without their port rows |
| click | select a node — the inspector shows its ports, live blackboard values and status history |
| right-click a node | force SUCCESS / FAILURE (control nodes too), stop a running motion, break before it starts, or change a leaf's policy |
| double-click a SubTree | collapse or expand it |
| Overrides ▾ | all actions manual or succeeding, cell leaves simulated again, clear overrides / breakpoints / edits, auto-release stalled gates |
| Cell sim | run the cell leaves with the cell-runner's semantics, or as stubs (starts the run over; for the tree shown — the next file starts out automatic again) |
| Cell | show or hide the cell panel |

A node's border, its incoming edge and the dot in its header carry the status, in Groot's monitor
colours: orange RUNNING, green SUCCESS, red FAILURE, and a muted shade for a node that has run and
was reset. `{blackboard}` port values are highlighted; hover a node to see what they resolve to.
Badges mark what holds a node (a stalled gate, a manual leaf, a breakpoint), an override, an
edited port and a motion's progress.

### Cell plans

When some cell leaf (`ExecuteTrajectory`, `MoveHome`, `WaitForRobot`, `CheckSafety`) has a
trajectory to follow, the cell leaves run with the cell-runner's semantics. Trees that only use
their names, like `v4_cell_with_gates.xml`, stay on stubs until you switch **Cell sim** on; that
choice is for the tree shown, so opening or reloading a file goes back to deciding by its
trajectories. While the cell leaves run, the builtins follow behaviortree-rs, the cell-runner's
runtime, where it differs from BT.CPP: a `SequenceStar` whose children all succeed returns
FAILURE there, and here too (the log says so).

* **Motion.** A motion leaf places its trajectory on the cell's schedule when it starts and stays
  RUNNING until the motion has played, held and settled; a `WaitForRobot` gate opens when the
  robot it waits for comes within tolerance. The **Cell** panel under the canvas shows the
  plan's ids, each robot's operation, phase, progress and joints, the motion placed so far as a
  Gantt chart (one lane per robot; click a bar to select its node, click or drag elsewhere to
  scrub), the warnings, and what the cell-runner would refuse to run in the file and the tree,
  first what it would report.
* **Halts.** When nothing can move without you — a gate that nothing will open, a manual leaf
  with nothing else scheduled, a breakpoint — the clock stops and a banner says so ("✋ Halted
  at t = 3.50 s (tick 12) — nothing moves until you decide"), with one row per waiting node:
  why it waits and the decisions that let it go on (**Release** / **Fail** a stalled gate,
  **Succeed** / **Fail** / **Simulate** a manual leaf, **Continue** / **Remove** a breakpoint).
  Its header offers **Release all stalled gates (like cell-runner)**. `cell_gate_stall.json`
  halts like this at 8.50 s.
* **Overrides.** Any node but the root can be forced to SUCCESS or FAILURE, once or pinned; a
  control node halts its children first. Forcing a running motion ("Stop motion now") cuts it
  where the robot is, marked ✂ in the Gantt chart. A leaf's policy switches between simulate
  (cell), wait for override, succeed and fail.
* **Breakpoints.** "Break before it starts" holds a node before it starts (it stays IDLE), once
  each time its parent starts — so a node directly under a `Repeat` or `Retry` pauses on the
  first pass only.
* **Edits.** A port's value is edited in place in the inspector's Ports grid (a number field, a
  checkbox or text; right-click it for *Type a value…*, e.g. a `{key}` pointer), a
  main-blackboard entry with its ✎. An edit takes effect the next time a node reads it; a
  running motion keeps what it started with. Edits survive Reset and can be reverted one by one
  (↺) or all at once; opening or reloading a file drops them.

## How it is put together

| crate | what it is | depends on egui |
|---|---|---|
| `bt-core` | XML → tree model, the builtin node manifest, Groot's layout, status traces | no |
| `bt-exec` | the BT.CPP 4 interpreter, blackboard scopes, the script language, stub and native leaves, overrides, breakpoints, edits, runs in virtual and real time | no |
| `bt-cell` | the cell-runner's plan formats, motion schedule and cell leaves, as native leaves of `bt-exec` | no |
| `bt-view` | `BtPanel`, everything drawn: the tree, the inspector, the halt banner, the Cell panel | yes |
| `btgui` | the window around `BtPanel`: reading files, the file picker | yes (eframe) |

### Fidelity

* **Parsing** follows Groot's `BuildTreeFromXML` / `ReadTreeNodesModel`: the model ID is the `ID`
  attribute or the tag, the instance name is `name` or the ID, every other attribute is a port
  value, and `<TreeNodesModel>` supplies the declared ports. BTCPP_format 4 additions (bare custom
  tags, `_skipIf`-style conditions, `{key}` SubTree remapping, `_autoremap`) are handled too, as is
  the v3 form where a `<SubTree>` attribute names the parent's key directly.
* **Layout** is a port of Groot's `RecursiveNodeReorder` (`bt_editor/utils.cpp`), including its
  `LEVEL_SPACING` of 80 and `NODE_SPACING` of 40, the way a parent is re-centred over its first and
  last child, and the per-level vertical stacking. Horizontal layout is the same with the axes
  swapped.
* **Status colours** are Groot's (`getStyleFromStatus`), including the muted shades for a node that
  went back to IDLE.
* **Execution** is our own interpreter rather than the `behaviortree-rs` crate the cell-runner uses,
  because that crate does not record a control node's status, inlines SubTrees away, has no
  status-change callback and is missing several v4 builtins — all four of which a monitor needs.
  The semantics (memory vs reactive, `Parallel` thresholds, halting running siblings, resetting
  children, the decorators' counters and timers) follow BT.CPP and are covered by
  `crates/bt-exec/tests/semantics.rs`; with the cell leaves installed they follow behaviortree-rs
  where it differs (`SequenceStar`, `crates/bt-cell/tests/runtime.rs`).
* **Cell leaves** follow the cell-runner (factoryos-orchestrator `cell/cell-runner`, `btree.rs` and
  `trajectory.rs`), whose plan parser, motion schedule and leaves `bt-cell` ports, each module
  citing the lines it comes from. Timings match: the welding sample ends at 75.2969 s and the
  two-robot one at 8.1479 s, as the cell-runner computes them; `crates/bt-cell/tests/parity.rs`
  ports the cell-runner's own schedule tests and `samples.rs` pins every step of both samples.
  Ports read as the cell-runner reads them (its Rust defaults, not `<TreeNodesModel>`'s), and the
  warnings are its own. Where a run you can interrupt needs a choice the cell-runner never makes,
  the default is what it does:
  * A motion the tree halts (a `Parallel` that has enough successes, say) plays to its end,
    because the cell-runner's motion leaves have no halt. Cutting it there is opt-in (the Cell
    panel's *When a parent halts a motion* combo). A motion you stop — an override on it or above it, or a policy
    switch — is always cut at that instant.
  * A gate nothing can open halts the run and says why, where the cell-runner releases every
    waiting gate with a "never reaches" warning. **Release all stalled gates (like
    cell-runner)** does that once, *Auto-release stalled gates* whenever the run stalls.
* **Deviations** from the cell-runner, on purpose:
  * The planner's JSON mirror (`root`) is converted to XML with a generated `<TreeNodesModel>`
    for the cell leaves, so they show typed ports with the cell-runner's defaults (no `name`
    port; `CheckSafety` is a condition) instead of unknown models.
  * `Delay` reads `delay_msec` as a float, like the cell-runner does (BT.CPP reads an integer):
    a number, or text that reads as one; anything else is 0.
  * What behaviortree-rs, the cell-runner's runtime, would refuse to build or tick is listed in
    the Cell panel as information, in the order it runs into them, and btgui runs the tree
    anyway: anything but the XML declaration before `<root>`, a `<BehaviorTree>` that is not one
    node with an end tag (a bare leaf, say), unknown tags (v3's `<Action ID>` included),
    undeclared attributes (`name` on a control node included), `_skipIf`-style conditions, a
    leaf written with an end tag or a control node without one, a missing SubTree, a node with
    the wrong number of children, a `ReactiveSequence` with more than one child that can be
    RUNNING and a `Parallel` whose thresholds exceed its children (R0–R6 in
    `crates/bt-cell/src/compat.rs`). Compiling a plan the way the cell-runner does
    (`bt_cell::compile`) fails on them with its error, word for word.
  * btgui warns about a motion that does not start where its robot is (*Jump*) or starts while
    the robot still moves (*Overlap*), and stops a runaway tree after 20 000 placed segments
    (`MAX_SEGMENTS`); the cell-runner has neither check.
  * Not ported: the cell-runner's playback (btgui scrubs its own history instead), collision
    checks and its server JSON. Cartesian gates are ignored, with the cell-runner's warning.

Not implemented: editing the tree's structure, the ZMQ/flatbuffers Groot monitor protocol, and the
full BT.CPP scripting language (assignments, comparisons, arithmetic, `? :` and the logical
operators are there; anything else logs a warning).

## Sharing a build

```
crates/btgui/scripts/package-macos.sh     # → dist/btgui-macos-<version>.zip
```

The zip holds a universal (Apple Silicon + Intel) `btgui.app` for macOS 11+, the sample trees and
plans in `examples/` (where the startup picker opens; the app holds a copy for when it runs
without that folder) and [packaging/README.txt](packaging/README.txt). It needs `rustup target
add x86_64-apple-darwin` once. The app is ad-hoc signed, not notarised, so first launch is
right-click → Open.

## Tests

```
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
cargo build --target wasm32-unknown-unknown -p bt-view   # keeps bt-view and bt-cell usable in the web viewer
```

`test_data/` holds Groot's own sample trees plus ours:

* `bt_1790934549_welding_cell_2_small.json` and `bt_1791274820_cell_2robot.json` — two plans as
  the planner writes them: two robots welding 15 operations (75.30 s), and one robot's staging
  move and the way home (8.15 s).
* `orchestrator_two_robots.xml` (the cell-runner's sample tree, byte for byte) and
  `orchestrator_two_robots.json` (its `btcpp_plan()` fixture). Opened in btgui, the XML picks up
  the JSON's trajectories and runs like the cell-runner (11.5 s); on its own (`load_text`) it
  runs on stubs.
* `cell_gate_stall.json` — robot2 waits for robot1 to park, at a pose 0.1 rad from where
  robot1's park move ends, so the gate never opens: Run to wait halts at 8.50 s with the gate in
  the banner. **Release** it (or **Release all stalled gates**, or tick *auto-release*) and the
  run ends at 16.50 s, as the cell-runner computes it.
* `v4_cell_with_gates.xml` — subtrees per robot, a reactive safety check, and a `WaitForRobot`
  gate. It has no trajectories, so its leaves are stubs that succeed after 1 s; with `--manual`
  (or *Overrides ▸ All actions manual*) each one, the gate included, waits for you to force it.

## Moving this into factoryos-orchestrator

The crates are written to be copied, not rewritten. `bt-core`, `bt-exec` and `bt-cell` have no UI
dependency, `bt-view` needs nothing but an `&mut egui::Ui` (no eframe, no `std::time`, no file
system, wasm-clean), and the versions in `[workspace.dependencies]` match what the orchestrator's
rerun 0.38.1 checkout resolves (egui 0.36.2).

1. Copy `crates/bt-core`, `crates/bt-exec`, `crates/bt-cell` and `crates/bt-view` into the
   orchestrator's `crates/` and add them to its workspace members. Drop `crates/btgui`.
   `bt-cell` is a port of the cell-runner's `btree.rs` / `trajectory.rs`; its parity tests are
   what keeps the two from drifting apart.
2. In `crates/factoryos-viewer/Cargo.toml`, add `bt-view = { path = "../bt-view" }`. The viewer
   reaches egui through `crate::rr`, so point `bt-view`'s egui dependency at the same re-export
   (`re_viewer::external::egui`) instead of the crates.io copy.
3. Add a `Page::BehaviourTree` to `app.rs`, route it in `page_ui`, and call `BtPanel::ui(ui)`.
   `sidebar.rs::executor_ui` already lists the cell's plans; a button there can load the selected
   plan's JSON into the panel with `BtPanel::load_text`. For a bare `.xml`, merge its sibling
   with `bt_cell::plan::merge_sibling` and call `BtPanel::load_plan`, as
   `crates/btgui/src/files.rs` does. `BtPanel::set_open_button` / `take_open_request` put an
   "Open…" on the empty canvas for the viewer's own file dialog.
4. btgui runs a plan with its own port of the cell-runner's semantics. For a live view of the
   *cell-runner's* execution instead, the cell-runner has to report per-node status: record each
   `set_status` in `cell/cell-runner/src/btree.rs` against virtual time, expose it next to
   `Playback::state_json`, and drive `BtPanel` from that trace instead of its own executor.
