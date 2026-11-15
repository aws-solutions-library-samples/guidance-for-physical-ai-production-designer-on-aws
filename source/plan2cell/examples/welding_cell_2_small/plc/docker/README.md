# Docker simulation for welding_cell_2_small

| Robot | Station | IP | Service | Gateway port |
|---|---|---|---|---|
| R1 robot1 | kuka-robi1 | 192.168.100.11 | device | 11741 |
| R2 robot2 | kuka-robi2 | 192.168.100.12 | device-r2 | 11741 |

Head PLC: `codesys-master` at 192.168.100.10 (gateway port 11740).

1. In `profinet-headplc-robot-simulation/docker`, copy `device-config` once per extra robot (see the comments in `docker-compose.robots.yml`).
2. `docker compose -f docker-compose.yml -f <out>/docker/docker-compose.robots.yml up -d --build`, then rerun the bridge settings from `install.sh` (multicast snooping off, promiscuous veths).
3. Download each `RobotSim_*` project to its container (by IP) before the head PLC project.

Each runtime hosts exactly one PROFINET device with a unique station name. Every container needs its own CODESYS runtime licence (or demo mode, which stops after a while).
