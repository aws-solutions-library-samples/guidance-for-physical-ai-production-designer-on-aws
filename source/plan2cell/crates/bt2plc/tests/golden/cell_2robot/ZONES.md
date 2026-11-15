# Interlock zones - cell_2robot

Plan `bt_1791274820_cell_2robot` (from `bt_1791274820_cell_2robot.json`), plan id 16#A312. 1 robot(s), 2 op(s), 0 cross-robot dependencies, 0 zone(s).

Every robot runs its own op list in one production Folge. Before an op that depends on another robot it raises the REQUEST bits of the op's zones, waits for the head PLC to GRANT all of them, runs the op, drops REQUEST and waits for GRANT to drop (4-phase handshake). The PLC grants each zone to one robot at a time **in ticket order**, which reproduces the behaviour tree's ordering exactly while letting BT-parallel ops run together.

## Zones (ticket order)

_none_

## Robot programs

- **robot1** (R1, Folge 11): STAGE_R1 HOME

## Dependencies

| Before | After | Enforced by |
|---|---|---|

## Schedule (nominal durations)

| Robot | # | Op | Zones | Grant (s) | End (s) | Expected (s) | Watchdog (s) |
|---|---|---|---|---|---|---|---|
| robot1 | 1 | STAGE_R1 | - | 0.000 | 3.729 | 3.729 | 10.594 |
| robot1 | 2 | HOME | - | 3.729 | 6.148 | 2.419 | 8.629 |

Makespan: **6.148 s**. Critical path: robot1.STAGE_R1 > robot1.HOME.
