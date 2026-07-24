# robot_ethercat_master_igh

External **IgH EtherCAT Master** package for [`robot_rt_task_control`](../robot_rt_task_control).

Hard-RT motion backend: **Timing + Job** own PDO; `Master::cycle()` only pushes setpoints.
Aligned capabilities with the EC-Master path (safe-output, skip-slot, realtime gates,
evidence gate, anomaly/dwell, CST freshness, DC monitor) — see [docs/INTEGRATION.md](docs/INTEGRATION.md).

## Boundary

| In this repo | Not in this repo |
|--------------|------------------|
| IgH servo / sync / CiA402 / kinematics / Job runtime | IgH kernel module & `/opt/etherlab` install |
| Stable C++ API: `ethercat_master_igh::Master` | OEM motor SKU binding in the RT task framework |
| Example motor YAML + `config/env.example` | Device-driver / realtime NIC setup |

## API (contract)

```text
init → map_joints → start → cycle (loop) → shutdown
```

Header: `ethercat_master_igh/master_api.hpp`

- `start()`: activate slaves → start Timing/Job (`SCHED_FIFO` + `mlockall`)
- `cycle()`: push setpoints + sample cached state (no PDO exchange)
- `request_safety_reset()`: clear fault latch + healthy dwell (no auto re-enable)

## Build

```bash
# Requires libethercat + ecrt.h (default /opt/etherlab)
# Optional: export ETHERLAB_ROOT=/opt/etherlab

cd /path/to/robot_ethercat_master_igh
source /opt/ros/humble/setup.bash
colcon build --packages-select ethercat_master_igh
```

Production: `LimitRTPRIO=99`, `LimitMEMLOCK=infinity`. Dev without privileges: `IGH_REQUIRE_REALTIME=0`.

## Layout

```text
src/ethercat_master_igh/
  include/ethercat_master_igh/master_api.hpp
  include/ethercat_joint/master/igh/igh_master_runtime.hpp
  src/master/igh/                              # sync + servo + runtime
  config/                                      # motor YAML + env.example
```
