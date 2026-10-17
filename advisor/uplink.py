"""Client-side mirror of AgentUplinkFilter.

The flight filter is still the guard. This mirror drops what must not be
written: any opcode other than AGENT.PROPOSE, FDIR.CLEAR, a sim time more
than 5 s behind, and a second proposal inside 2 s of wall time.

A batch carries at most the first command. The second opcode is not framed
and is not used as a fallback. Numeric opcode values are not hardcoded.
"""

from __future__ import annotations

import json
import math
import socket

HOST = "127.0.0.1"
PORT = 50001
INTERVAL_S = 2.0
MAX_LAG_NS = 5_000_000_000
PROPOSE = "AGENT.PROPOSE"
CLEAR = "FDIR.CLEAR"
ACTIONS = ("NOMINAL", "ISOLATE_RW", "MOMENTUM_DUMP", "LOAD_SHED", "SAFE")


def _wheel_index(value: object) -> int | None:
    if isinstance(value, bool) or not isinstance(value, int):
        return None
    if value < 0 or value > 3:
        return None
    return value


def _confidence(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    if not math.isfinite(value) or value < 0.0 or value > 1.0:
        return None
    return float(value)


class UplinkMirror:
    def __init__(self, *, interval_s: float = INTERVAL_S, max_lag_ns: int = MAX_LAG_NS) -> None:
        self.interval_s = interval_s
        self.max_lag_ns = max_lag_ns
        self._last_accept_wall: float | None = None

    def frame(self, commands: object, *, current_sim_time_ns: object, wall_time: float) -> dict | None:
        """Frame one AGENT.PROPOSE, or return None. Does not open a socket."""
        if isinstance(commands, dict):
            commands = [commands]
        if not isinstance(commands, list) or not commands:
            return None
        command = commands[0]
        if not isinstance(command, dict):
            return None
        opcode = command.get("opcode")
        if opcode == CLEAR or opcode != PROPOSE:
            return None
        action = command.get("action")
        wheel_index = _wheel_index(command.get("wheel_index"))
        confidence = _confidence(command.get("confidence"))
        sim_time_ns = command.get("sim_time_ns")
        if action not in ACTIONS or wheel_index is None or confidence is None:
            return None
        if isinstance(sim_time_ns, bool) or not isinstance(sim_time_ns, int):
            return None
        if isinstance(current_sim_time_ns, bool) or not isinstance(current_sim_time_ns, int):
            return None
        if not isinstance(wall_time, (int, float)) or isinstance(wall_time, bool):
            return None
        if current_sim_time_ns - sim_time_ns > self.max_lag_ns:
            return None
        if self._last_accept_wall is not None:
            if wall_time - self._last_accept_wall < self.interval_s:
                return None
        framed = {
            "opcode": PROPOSE,
            "action": action,
            "wheel_index": wheel_index,
            "confidence": confidence,
            "sim_time_ns": sim_time_ns,
        }
        self._last_accept_wall = float(wall_time)
        return framed


def send_frame(frame: dict, *, timeout: float = 1.0) -> None:
    """Write one JSON command to 127.0.0.1:50001.

    Does not open the Basilisk sensor port, the mode port, or the GDS port.
    Callers must already hold an accepted AGENT.PROPOSE frame.
    """
    if not isinstance(frame, dict) or frame.get("opcode") != PROPOSE:
        raise ValueError("refusing to send a frame that is not AGENT.PROPOSE")
    payload = (json.dumps(frame, separators=(",", ":")) + "\n").encode("utf-8")
    with socket.create_connection((HOST, PORT), timeout=timeout) as sock:
        sock.sendall(payload)
