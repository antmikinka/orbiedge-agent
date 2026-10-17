# Bus FDIR plant

Basilisk, or the canned thermal script, owns the dynamics. F´ `FdirManager` is the only mode authority. This directory does not speak the 256-byte `bsk.state` packet used by `TelemetryMonitor`.

| Port | Role |
| --- | --- |
| `127.0.0.1:5600` | Sensor JSON. The plant is the server. `SimBridge` is the client. |
| `127.0.0.1:5601` | Mode JSON. F´ is the server. The plant is the client. |

Sensor line: `sim_time_ns`, `soc`, `solar_scale`, `wheel_speed_rad_s` (4), `temp_c`, `heater_on`.

Mode line: `sim_time_ns`, `mode`, `wheel_index`.

| Mode | Plant effect |
| --- | --- |
| `NOMINAL` | Stand-in MRP wheel controller. |
| `ISOLATE_RW` | Named wheel torque held at 0. |
| `MOMENTUM_DUMP` | Scripted torque opposing wheel speed. The power example has no magnetorquer. |
| `LOAD_SHED` | Wheel torque scaled to a quarter. |
| `SAFE` | All wheel torques 0, heater flag cleared, no further slew. |

`ISOLATE_RW` and `SAFE` stay until a human `FDIR.CLEAR`. The advisor is not started here.

Phase 1, with the F´ deployment already running and the advisor absent:

```text
python sim/fdir/canned_plant.py --family thermal-critical
```

A passing run prints `phase1 SAFE sim_time_ns=<n> heater_cleared=1`.
