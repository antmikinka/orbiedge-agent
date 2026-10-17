"""Thermal critical canned run.

Temperature ramps from 20 C to 70 C with the heater on, then holds at
70 C so the 50-sample critical persistence is met. SOC stays at 0.8 and
the wheels stay at rest. No proposal is offered.

The default path is in-process and does not bind a port. ``--live`` binds
127.0.0.1:5600 as the sensor publisher and connects to 127.0.0.1:5601 for
mode lines. The in-process manager is used only when that peer is absent.
"""

from __future__ import annotations

import sys

from sim.fdir.rules import STEP_NS, TEMP_CRIT_SAMPLES, BusMonitor, FdirManager, sensor_sample
from sim.fdir.wire import format_sensor_line, parse_mode_line

_RAMP_STEPS = 51  # 20 C, 21 C, ... 70 C


def _thermal_samples() -> list[dict]:
    samples: list[dict] = []
    index = 0
    for step in range(_RAMP_STEPS):
        samples.append(
            sensor_sample(
                index * STEP_NS,
                soc=0.8,
                solar_scale=1.0,
                wheel_speed_rad_s=(0.0, 0.0, 0.0, 0.0),
                temp_c=20.0 + float(step),
                heater_on=True,
            )
        )
        index += 1
    for _ in range(TEMP_CRIT_SAMPLES):
        samples.append(
            sensor_sample(
                index * STEP_NS,
                soc=0.8,
                solar_scale=1.0,
                wheel_speed_rad_s=(0.0, 0.0, 0.0, 0.0),
                temp_c=70.0,
                heater_on=True,
            )
        )
        index += 1
    return samples


def _run_inprocess(samples: list[dict], subscriber=None) -> list[str]:
    monitor = BusMonitor()
    manager = FdirManager(monitor)
    timeline: list[str] = []
    for sample in samples:
        if subscriber is not None:
            subscriber.sendall(format_sensor_line(sample).encode("utf-8"))
        record = monitor.ingest(sample)
        decision = manager.tick(record, None, True)
        timeline.append(decision.mode)
    return timeline


def _read_line(sock, pending: bytes) -> tuple[bytes, bytes]:
    while b"\n" not in pending:
        chunk = sock.recv(4096)
        if not chunk:
            raise ConnectionError("mode peer closed before a mode line")
        pending += chunk
    line, _, pending = pending.partition(b"\n")
    return line, pending


def _run_peer(samples: list[dict], subscriber, peer) -> list[str]:
    timeline: list[str] = []
    pending = b""
    for sample in samples:
        if subscriber is not None:
            subscriber.sendall(format_sensor_line(sample).encode("utf-8"))
        line, pending = _read_line(peer, pending)
        mode_obj = parse_mode_line(line)
        timeline.append(mode_obj["mode"])
    return timeline


def _run_live(samples: list[dict]) -> list[str]:
    import socket

    peer = None
    try:
        peer = socket.create_connection(("127.0.0.1", 5601), timeout=0.3)
        peer.settimeout(0.3)
    except OSError:
        peer = None

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    subscriber = None
    try:
        server.bind(("127.0.0.1", 5600))
        server.listen(1)
        server.settimeout(0.3)
        try:
            subscriber, _addr = server.accept()
            subscriber.settimeout(0.3)
        except OSError:
            subscriber = None
        if peer is None:
            return _run_inprocess(samples, subscriber)
        return _run_peer(samples, subscriber, peer)
    finally:
        if subscriber is not None:
            subscriber.close()
        if peer is not None:
            peer.close()
        server.close()


def run(live: bool = False) -> list[str]:
    """Return the mode after each sample. SAFE once critical persistence is met."""
    samples = _thermal_samples()
    timeline = _run_live(samples) if live else _run_inprocess(samples)
    if "SAFE" not in timeline:
        raise RuntimeError("thermal critical persistence did not reach SAFE")
    return timeline


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    timeline = run(live="--live" in args)
    sys.stdout.write("\n".join(timeline) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
