"""JSON lines for the 10 Hz sensor bus and the mode bus.

One JSON object per line. Unknown mode names are rejected.
"""

from __future__ import annotations

import json

from sim.fdir.rules import MODES, WHEEL_COUNT

_SENSOR_KEYS = (
    "sim_time_ns",
    "soc",
    "solar_scale",
    "wheel_speed_rad_s",
    "temp_c",
    "heater_on",
)


def _object(line: str | bytes) -> dict:
    if isinstance(line, bytes):
        line = line.decode("utf-8")
    text = line.strip()
    if not text:
        raise ValueError("empty line")
    obj = json.loads(text)
    if not isinstance(obj, dict):
        raise ValueError("expected one JSON object")
    return obj


def parse_sensor_line(line: str | bytes) -> dict:
    """Parse one sensor object. Extra keys are ignored."""
    obj = _object(line)
    missing = [key for key in _SENSOR_KEYS if key not in obj]
    if missing:
        raise ValueError(f"sensor missing {missing[0]}")
    sim_time_ns = obj["sim_time_ns"]
    wheels = obj["wheel_speed_rad_s"]
    if isinstance(sim_time_ns, bool) or not isinstance(sim_time_ns, int):
        raise ValueError("sim_time_ns must be an int")
    if not isinstance(wheels, list) or len(wheels) != WHEEL_COUNT:
        raise ValueError(f"wheel_speed_rad_s needs {WHEEL_COUNT} values")
    if not isinstance(obj["heater_on"], bool):
        raise ValueError("heater_on must be a bool")
    return {
        "sim_time_ns": sim_time_ns,
        "soc": float(obj["soc"]),
        "solar_scale": float(obj["solar_scale"]),
        "wheel_speed_rad_s": [float(speed) for speed in wheels],
        "temp_c": float(obj["temp_c"]),
        "heater_on": obj["heater_on"],
    }


def format_sensor_line(sample: dict) -> str:
    """Serialize one sensor object, including the trailing newline."""
    parsed = parse_sensor_line(json.dumps(sample))
    return json.dumps(parsed, separators=(",", ":")) + "\n"


def parse_mode_line(line: str | bytes) -> dict:
    """Parse one mode object. Unknown modes raise ValueError."""
    obj = _object(line)
    try:
        mode = obj["mode"]
        sim_time_ns = obj["sim_time_ns"]
        wheel_index = obj["wheel_index"]
    except KeyError as exc:
        raise ValueError(f"mode missing {exc.args[0]}") from exc
    if mode not in MODES:
        raise ValueError(f"unknown mode: {mode}")
    if isinstance(sim_time_ns, bool) or not isinstance(sim_time_ns, int):
        raise ValueError("sim_time_ns must be an int")
    if isinstance(wheel_index, bool) or not isinstance(wheel_index, int):
        raise ValueError("wheel_index must be an int")
    if wheel_index != -1 and not 0 <= wheel_index < WHEEL_COUNT:
        raise ValueError("wheel_index out of range")
    return {
        "sim_time_ns": sim_time_ns,
        "mode": mode,
        "wheel_index": wheel_index,
    }


def format_mode_line(mode_obj: dict) -> str:
    """Serialize one mode object. Unknown modes raise ValueError."""
    parsed = parse_mode_line(json.dumps(mode_obj))
    return json.dumps(parsed, separators=(",", ":")) + "\n"
