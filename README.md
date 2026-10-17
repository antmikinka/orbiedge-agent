# orbiedge-agent

Basilisk is the physical truth model for a 3U CubeSat in low Earth orbit. F Prime receives that state. It does not fly the vehicle.

The stand-in attitude controller stays in the Basilisk script: `inertial3D`, `attTrackingError`, `mrpFeedback`, and `rwMotorTorque`. `TelemetryMonitor` subscribes to a ZeroMQ state packet, checks it, and writes F Prime telemetry. It has no actuator port.

## Layout

| Path | Role |
| --- | --- |
| `sim/scenarios/leo_3u_nominal.py` | 3U LEO scenario, three physical faults, ZeroMQ publisher |
| `sim/bridge/packet.py` | 256-byte `BSK1` packet |
| `Orbiedge/TelemetryMonitor/` | F Prime active component |
| `third_party/fprime` | submodule, [nasa/fprime](https://github.com/nasa/fprime) `devel` |
| `third_party/basilisk` | submodule, [AVSLab/basilisk](https://github.com/AVSLab/basilisk) `develop` |

The checklist name `avs-simulation/basilisk` is not a GitHub organization. The pin is `AVSLab/basilisk`. `scripts/init_submodules.ps1` adds both checkouts with `--depth 1`. Their source is not copied into `sim/` or `Orbiedge/`.

## Scenario

`sim/scenarios/leo_3u_nominal.py` builds one process and one task at 0.5 s.

- Orbit: 400 km altitude, eccentricity 0, inclination 51.6 deg. Epoch `2026 JAN 01 00:00:00.000 (UTC)`.
- Mass: 4.39 kg. That is a 3.6 kg bus, two 0.20 kg panels, and three 0.130 kg BCT RWP015 wheels. `BalancedWheels` does not add wheel mass, so the wheels are in `mHub`.
- Inertia about the body center: `diag(0.050, 0.032, 0.025)` kg m^2. MRP gains are `K = 0.008`, `P = 0.08`, `Ki = -1`. The stock example gains belong to a much larger spacecraft.
- Gravity: GGM03S degree 2. The run stops if that coefficient file cannot be resolved.
- Drag: `MsisAtmosphere` (NRLMSISE-00) into `DragDynamicEffector`. Cannonball area 0.03 m^2, Cd 2.2. Quiet space weather is Ap 10 and F10.7 140.
- SRP: two rigid facets, area 0.03 m^2, centers of pressure at `y = ±0.20` m.
- Magnetic field: `MagneticFieldWMM` with `WMM2025.COF`. Basilisk does not ship a separate IGRF class. WMM is the model this scenario runs.
- Actuators: three orthogonal BCT RWP015 wheels. RW3 is body +Z, index 2. Four MOOG Monarc-1 thrusters on the −Z face are attached with a zero on-time command.
- Sensors: `simpleNav` for the control loop, plus an IMU, a star tracker, and four coarse sun sensors. The exported gyro is the IMU output.

Fault schedule, measured at the end of each step:

| Time | Fault | Flag bit |
| --- | --- | --- |
| 120 s | RW3 torque clipped to 0.2 mNm plus a 0.15 mNm friction spike | 0 |
| 240 s | Both panel efficiencies set to 40 percent of 0.28 | 1 |
| 360 s | Gyro bias random walk, 1e-4 rad/s/sqrt(s), written to `senRotBias` | 2 |

`senRotBias` is added inside `ImuSensor` on every update. The Python `Rw3Stiction` module is a Basilisk `SysModel` director. If a build has directors disabled, the stiction flag is still set and the torque tap does not run.

`SimpleBattery` publishes stored charge and net power. It does not publish voltage. The packet field is a stand-in:

`V = 8.0 * (0.85 + 0.15 * clamp(soc, 0, 1))`

```text
pip install "bsk[all]" pyzmq numpy
python sim/scenarios/leo_3u_nominal.py --duration 600
python sim/scenarios/leo_3u_nominal.py --duration 30 --no-faults --no-zmq
```

The publisher binds `tcp://127.0.0.1:5555`, waits 0.2 s, then sends multipart frames `[b"bsk.state", packet]`. A full send queue drops the sample. The first packets can also be dropped before a subscriber connects.

A 121 s `--no-zmq` run on Basilisk 2.12.0 completed 242 steps. The MRP moved from `[0.05, -0.03, 0.02]` to about `[-0.0019, -0.002, 0.0081]`. State of charge went from 0.900 to 0.905, and the derived bus voltage from 7.880 V to 7.886 V. At 120.5 s the fault flags became `0x0001`, the RW3 stiction bit, and the run continued. The 240 s brownout and the 360 s IMU walk were not reached. A separate 2 s run bound the publisher to `tcp://127.0.0.1:5555`. No subscriber was attached. `setUseApHistory` and `setPlanetPolarRadius` are absent from the 2.12 binding. The script calls them only when the installed module has them. The constructor already sets Earth's equatorial radius and the daily-Ap switch.

## Packet

256 bytes, little-endian, version 1. Magic `0x42534B31` (`BSK1`). CRC-32 (the same polynomial as `zlib.crc32`) covers the first 252 bytes.

| Offset | Field |
| --- | --- |
| 0 | magic `u32` |
| 4 | version `u16` = 1 |
| 6 | fault flags `u16` |
| 8 | `simTimeNs` `u64` |
| 16 | `q_BN[4]` `f64`, scalar first |
| 48 | `omega_BN_B[3]` `f64`, rad/s |
| 72 | derived bus voltage `f64`, V |
| 80 | battery net power `f64`, W |
| 88 | state of charge `f64` |
| 96 | wheel speeds `f64[3]`, rad/s |
| 120 | IMU `AngVelPlatform` `f64[3]`, rad/s |
| 144 | star-tracker quaternion `f64[4]`, scalar first |
| 176 | four CSS outputs `f64[4]` |
| 208 | panel efficiency `f64` |
| 216 | neutral density `f64`, kg/m^3 |
| 224 | sequence `u32` |
| 228 | 24 zero pad bytes |
| 252 | CRC-32 `u32` |

`python tests/test_packet.py` checks the Python codec. `tests/test_packet_unpack.cpp` decodes `tests/golden/bsk_state_v1.bin` with the shared header and does not link F Prime or ZeroMQ.

## TelemetryMonitor

`Orbiedge/TelemetryMonitor` is an F Prime active component.

- `START <endpoint>` opens a ZeroMQ `SUB` socket, subscribes to `bsk.state`, and sets a zero receive timeout.
- `STOP` closes it.
- `schedIn` is an async port with `drop`. The handler runs on the component thread, so the ZeroMQ socket is not shared with the rate group. A full queue drops the tick instead of blocking the caller. The poll itself does not block.
- An accepted packet updates `SimTimeNs`, `q_BN`, `omega_BN_B`, `BusVoltage`, `WheelSpeeds`, `FaultFlags`, and `PacketsAccepted`, emits `PacketAccepted`, and invokes `stateOut` when that port is connected.
- A bad size, magic, version, or CRC emits `PacketRejected` and does not change the last good telemetry.

`settings.ini` points the framework path at `third_party/fprime`. There is no deployment in this repository yet, so the build produces the static library `libOrbiedge_TelemetryMonitor.a`. It does not produce a bootable binary. A later topology wires `schedIn` to a rate group, the command ports to the command dispatcher, and `stateOut` to whatever records the sample.

F´ `devel` has no Windows platform. Configure on Linux or on WSL Ubuntu. CMake must be 3.26 or newer (`/usr/bin/cmake` on Ubuntu 24.04 is 3.28). The depth-1 F´ pin does not include googletest, so leave `BUILD_TESTING` off. The module requires libzmq (`sudo apt install libzmq3-dev`, or pass `CMAKE_PREFIX_PATH` to a libzmq 4.3 install). The Python tools used here are `fprime-tools==4.4.0`, `fprime-fpp==3.4.0`, and `fprime-jre==25.0.4.1.1`, with `JAVA_HOME` pointed at that JRE.

```bash
export PATH="$HOME/orbiedge-fprime-venv/bin:/usr/bin:/bin"
export JAVA_HOME="$HOME/orbiedge-fprime-venv/lib/python3.12/site-packages/fprime_jre/runtime"
export PATH="$JAVA_HOME/bin:$PATH"
cmake -S . -B "$HOME/orbiedge-build" -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$HOME/opt/libzmq" \
  -DBUILD_TESTING=OFF \
  -DFPRIME_ENABLE_FRAMEWORK_UTS=OFF \
  -DFPRIME_ENABLE_AUTOCODER_UTS=OFF
cmake --build "$HOME/orbiedge-build" --target Orbiedge_TelemetryMonitor
```

That build completed on Ubuntu 24.04 in WSL and linked `zmq_connect`, `zmq_msg_recv`, and `zmq_setsockopt` into `Orbiedge_TelemetryMonitor`. The component does not link Basilisk.

## What this stub leaves out

A separate OrbiEdge architecture note describes a later bridge: 10 Hz JSON on `127.0.0.1:5600` and `:5601`, and F Prime components named `SimBridge`, `BusMonitor`, and `FdirManager`, plus a Windows NPU advisor. That topology is not in this repository. This tree is the execution checklist: the public submodule pins, the 0.5 s Basilisk scenario, and `TelemetryMonitor`.
