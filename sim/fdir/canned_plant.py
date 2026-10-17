"""Phase 1 thermal plant. Standard library only. No Basilisk and no advisor.

TCP server 127.0.0.1:5600 publishes sensor JSON at 10 Hz.
TCP client 127.0.0.1:5601 reads mode JSON. Ports 50000 and 50001 are not used.
"""

from __future__ import annotations

import argparse
import json
import socket
import time

HOST = "127.0.0.1"
STEP_NS = 100_000_000
CRIT_C = 60.0
PERSIST_NS = 5_000_000_000
FAIL_AFTER_NS = 30_000_000_000


def thermal_temp_c(sim_time_ns: int) -> float:
    return 20.0 + 2.0 * (sim_time_ns / 1_000_000_000)


def _bind_server(port: int) -> socket.socket:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, port))
    server.listen(2)
    server.setblocking(False)
    return server


def _try_mode(port: int, current: socket.socket | None) -> socket.socket | None:
    if current is not None:
        return current
    try:
        client = socket.create_connection((HOST, port), timeout=0.05)
    except OSError:
        return None
    client.setblocking(False)
    return client


def _read_mode(client: socket.socket | None, buffer: bytearray) -> tuple[socket.socket | None, dict | None]:
    if client is None:
        return None, None
    try:
        chunk = client.recv(4096)
    except BlockingIOError:
        return client, None
    except OSError:
        return None, None
    if chunk == b"":
        return None, None
    buffer.extend(chunk)
    if b"\n" not in buffer:
        return client, None
    line, _, rest = buffer.partition(b"\n")
    buffer[:] = rest
    try:
        payload = json.loads(line.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError):
        return client, None
    if not isinstance(payload, dict):
        return client, None
    return client, payload


def run_thermal(sensor_port: int, mode_port: int) -> int:
    server = _bind_server(sensor_port)
    clients: list[socket.socket] = []
    mode_client = None
    mode_buffer = bytearray()
    sim_time_ns = 0
    above_since = None
    persist_done = None
    next_send = time.monotonic()
    try:
        while True:
            try:
                client, _addr = server.accept()
            except BlockingIOError:
                client = None
            if client is not None:
                client.setblocking(False)
                clients.append(client)
            mode_client = _try_mode(mode_port, mode_client)
            mode_client, payload = _read_mode(mode_client, mode_buffer)
            now = time.monotonic()
            if clients and now >= next_send:
                temp_c = thermal_temp_c(sim_time_ns)
                heater_on = True
                if temp_c > CRIT_C and above_since is None:
                    above_since = sim_time_ns
                if above_since is not None and sim_time_ns - above_since >= PERSIST_NS and persist_done is None:
                    persist_done = sim_time_ns
                sample = {
                    "sim_time_ns": sim_time_ns,
                    "soc": 0.8,
                    "solar_scale": 1.0,
                    "wheel_speed_rad_s": [0, 0, 0, 0],
                    "temp_c": temp_c,
                    "heater_on": heater_on,
                }
                encoded = (json.dumps(sample, separators=(",", ":")) + "\n").encode("utf-8")
                living = []
                for client in clients:
                    try:
                        client.sendall(encoded)
                    except OSError:
                        client.close()
                        continue
                    living.append(client)
                clients = living
                sim_time_ns += STEP_NS
                next_send = now + 0.1
            if payload is not None and payload.get("mode") == "SAFE" and persist_done is not None:
                cleared = {
                    "sim_time_ns": sim_time_ns,
                    "soc": 0.8,
                    "solar_scale": 1.0,
                    "wheel_speed_rad_s": [0, 0, 0, 0],
                    "temp_c": thermal_temp_c(sim_time_ns),
                    "heater_on": False,
                }
                encoded = (json.dumps(cleared, separators=(",", ":")) + "\n").encode("utf-8")
                for client in clients:
                    try:
                        client.sendall(encoded)
                    except OSError:
                        pass
                mode_time = payload.get("sim_time_ns")
                print("phase1 SAFE sim_time_ns={} heater_cleared=1".format(mode_time))
                return 0
            if persist_done is not None and sim_time_ns - persist_done >= FAIL_AFTER_NS:
                print("phase1 FAIL")
                return 1
            time.sleep(0.01)
    finally:
        for client in clients:
            client.close()
        if mode_client is not None:
            mode_client.close()
        server.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Canned Phase 1 thermal plant")
    parser.add_argument("--family", default="thermal-critical")
    parser.add_argument("--duration", type=float, default=40.0)
    parser.add_argument("--sensor-port", type=int, default=5600)
    parser.add_argument("--mode-port", type=int, default=5601)
    args = parser.parse_args(argv)
    if args.family != "thermal-critical":
        print("phase1 FAIL")
        return 1
    if args.sensor_port in (50000, 50001) or args.mode_port in (50000, 50001):
        print("phase1 FAIL")
        return 1
    del args.duration
    return run_thermal(args.sensor_port, args.mode_port)


if __name__ == "__main__":
    raise SystemExit(main())
