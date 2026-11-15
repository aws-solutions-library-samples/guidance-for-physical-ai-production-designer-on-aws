# Interlock zones - welding_cell_2_small

Plan `bt_1790934549_welding_cell_2_small` (from `bt_1790934549_welding_cell_2_small.json`), plan id 16#F77E. 2 robot(s), 18 op(s), 13 cross-robot dependencies, 2 zone(s).

Every robot runs its own op list in one production Folge. Before an op that depends on another robot it raises the REQUEST bits of the op's zones, waits for the head PLC to GRANT all of them, runs the op, drops REQUEST and waits for GRANT to drop (4-phase handshake). The PLC grants each zone to one robot at a time **in ticket order**, which reproduces the behaviour tree's ordering exactly while letting BT-parallel ops run together.

## Zones (ticket order)

- **IL01** (bit 0): robot1.W01 > robot2.W15 > robot1.W03 > robot2.W10 > robot1.W05 > robot2.W12 > robot1.R1_PARK > robot2.W09
- **IL02** (bit 1): robot2.W13 > robot1.W02 > robot2.W14 > robot1.W06 > robot2.W11 > robot1.W04 > robot2.W07 > robot1.HOME

## Robot programs

- **robot1** (R1, Folge 11): W01[IL01] W02[IL02] W03[IL01] W06[IL02] W05[IL01] W04[IL02] R1_PARK[IL01] HOME[IL02]
- **robot2** (R2, Folge 11): W13[IL02] W15[IL01] W14[IL02] W10[IL01] W11[IL02] W12[IL01] W09[IL01] W08 W07[IL02] HOME

## Dependencies

| Before | After | Enforced by |
|---|---|---|
| robot2.W13 | robot1.W02 | IL02 |
| robot1.W01 | robot2.W15 | IL01 |
| robot2.W15 | robot1.W03 | IL01 |
| robot1.W02 | robot2.W14 | IL02 |
| robot2.W14 | robot1.W06 | IL02 |
| robot1.W03 | robot2.W10 | IL01 |
| robot2.W10 | robot1.W05 | IL01 |
| robot1.W06 | robot2.W11 | IL02 |
| robot2.W11 | robot1.W04 | IL02 |
| robot1.W05 | robot2.W12 | IL01 |
| robot2.W12 | robot1.R1_PARK | IL01 |
| robot1.R1_PARK | robot2.W09 | IL01 |
| robot2.W07 | robot1.HOME | IL02 |

## Schedule (nominal durations)

| Robot | # | Op | Zones | Grant (s) | End (s) | Expected (s) | Watchdog (s) |
|---|---|---|---|---|---|---|---|
| robot1 | 1 | W01 | IL01 | 0.000 | 11.132 | 11.132 | 21.698 |
| robot1 | 2 | W02 | IL02 | 11.132 | 15.453 | 4.321 | 11.482 |
| robot1 | 3 | W03 | IL01 | 19.557 | 25.508 | 5.951 | 13.927 |
| robot1 | 4 | W06 | IL02 | 25.508 | 28.180 | 2.672 | 9.008 |
| robot1 | 5 | W05 | IL01 | 30.500 | 36.412 | 5.912 | 13.868 |
| robot1 | 6 | W04 | IL02 | 36.412 | 40.962 | 4.550 | 11.825 |
| robot1 | 7 | R1_PARK | IL01 | 41.233 | 43.904 | 2.671 | 9.007 |
| robot1 | 8 | HOME | IL02 | 55.966 | 64.297 | 8.331 | 17.497 |
| robot2 | 1 | W13 | IL02 | 0.000 | 9.686 | 9.686 | 19.529 |
| robot2 | 2 | W15 | IL01 | 11.132 | 19.557 | 8.425 | 17.638 |
| robot2 | 3 | W14 | IL02 | 19.557 | 24.281 | 4.724 | 12.086 |
| robot2 | 4 | W10 | IL01 | 25.508 | 30.500 | 4.992 | 12.488 |
| robot2 | 5 | W11 | IL02 | 30.500 | 35.536 | 5.036 | 12.554 |
| robot2 | 6 | W12 | IL01 | 36.412 | 41.233 | 4.821 | 12.232 |
| robot2 | 7 | W09 | IL01 | 43.904 | 46.603 | 2.699 | 9.048 |
| robot2 | 8 | W08 | - | 46.603 | 51.197 | 4.594 | 11.891 |
| robot2 | 9 | W07 | IL02 | 51.197 | 55.966 | 4.769 | 12.154 |
| robot2 | 10 | HOME | - | 55.966 | 62.195 | 6.229 | 14.344 |

Makespan: **64.297 s**. Critical path: robot1.W01 > robot2.W15 > robot1.W03 > robot2.W10 > robot1.W05 > robot2.W12 > robot1.R1_PARK > robot2.W09 > robot2.W08 > robot2.W07 > robot1.HOME.
