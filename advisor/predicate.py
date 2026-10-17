"""Predicate summary for the advisor prompt.

Scalars are always present. Clauses are the fault record's violations,
capped at 16 so the short-context model is the one that would run.
"""

from __future__ import annotations

import math

MAX_CLAUSES = 16

_SCALAR_KEYS = (
    "mode",
    "sim_time_ns",
    "soc",
    "solar_scale",
    "max_wheel_speed",
    "max_temp_c",
    "eligible",
    "default_mode",
)


def _as_dict(value: object) -> dict:
    return value if isinstance(value, dict) else {}


def _finite(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    if not math.isfinite(value):
        return None
    return float(value)


def _first_finite(sample: dict, fault: dict, keys: tuple[str, ...]) -> float | None:
    for source in (sample, fault):
        for key in keys:
            number = _finite(source.get(key))
            if number is not None:
                return number
    return None


def _sim_time_ns(sample: dict, fault: dict) -> int | None:
    for source in (sample, fault):
        value = source.get("sim_time_ns")
        if isinstance(value, int) and not isinstance(value, bool):
            return value
    return None


def _text(sample: dict, fault: dict, key: str) -> str | None:
    for source in (sample, fault):
        value = source.get(key)
        if isinstance(value, str):
            return value
    return None


def _eligible(sample: dict, fault: dict) -> list[str]:
    for source in (fault, sample):
        value = source.get("eligible")
        if isinstance(value, list):
            return [item for item in value if isinstance(item, str)]
    return []


def _max_wheel_speed(sample: dict, fault: dict) -> float | None:
    explicit = _first_finite(sample, fault, ("max_wheel_speed",))
    if explicit is not None:
        return explicit
    speeds = None
    for source in (sample, fault):
        for key in ("wheel_speed_rad_s", "wheel_speeds"):
            value = source.get(key)
            if isinstance(value, list):
                speeds = value
                break
        if speeds is not None:
            break
    if not speeds:
        return None
    magnitudes = [abs(number) for item in speeds if (number := _finite(item)) is not None]
    if not magnitudes:
        return None
    return float(max(magnitudes))


def _explicit_clauses(fault: dict, sample: dict) -> list:
    for source in (fault, sample):
        for key in ("clauses", "violations"):
            value = source.get(key)
            if isinstance(value, list):
                return value
    return []


def _normalize_clause(item: object) -> dict | None:
    if not isinstance(item, dict):
        return None
    channel = item.get("channel")
    if not isinstance(channel, str) or not channel:
        return None
    value = _finite(item.get("value"))
    limit = _finite(item.get("limit"))
    persistence = item.get("persistence")
    if value is None or limit is None:
        return None
    if isinstance(persistence, bool) or not isinstance(persistence, (int, float)):
        return None
    if isinstance(persistence, float) and not math.isfinite(persistence):
        return None
    return {
        "channel": channel,
        "value": value,
        "limit": limit,
        "persistence": int(persistence) if isinstance(persistence, int) else persistence,
    }


def build_summary(sample: object, fault: object) -> dict:
    """Build the predicate summary. Always includes the scalar fields.

    More than 16 clauses are truncated. The tail is discarded before
    normalization so a long fault record cannot grow the prompt.
    """
    sample_d = _as_dict(sample)
    fault_d = _as_dict(fault)
    raw = _explicit_clauses(fault_d, sample_d)[:MAX_CLAUSES]
    clauses = []
    for item in raw:
        clause = _normalize_clause(item)
        if clause is not None:
            clauses.append(clause)
    summary = {
        "mode": _text(sample_d, fault_d, "mode"),
        "sim_time_ns": _sim_time_ns(sample_d, fault_d),
        "soc": _first_finite(sample_d, fault_d, ("soc",)),
        "solar_scale": _first_finite(sample_d, fault_d, ("solar_scale",)),
        "max_wheel_speed": _max_wheel_speed(sample_d, fault_d),
        "max_temp_c": _first_finite(sample_d, fault_d, ("max_temp_c", "temp_c")),
        "eligible": _eligible(sample_d, fault_d),
        "default_mode": _text(sample_d, fault_d, "default_mode"),
        "clauses": clauses,
    }
    missing = [key for key in _SCALAR_KEYS if key not in summary]
    if missing:
        raise RuntimeError("summary missing scalars: " + ",".join(missing))
    return summary
