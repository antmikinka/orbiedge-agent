"""Five FDIR families, twenty seeds each. All runs are in-process.

Seeds change onset and magnitude only. A nominal seed never arms a fault.
The summary is written to tests/golden/fdir_matrix_summary.json.
The process exits non-zero when any run misses its expected mode.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

from sim.fdir.rules import (
    STEP_NS,
    WHEEL_COUNT,
    WHEEL_MAX_RAD_S,
    BusMonitor,
    FdirManager,
    sensor_sample,
)

ROOT = Path(__file__).resolve().parents[2]
SUMMARY_PATH = ROOT / "tests" / "golden" / "fdir_matrix_summary.json"

FAMILIES = ("nominal", "rw_saturation", "rw_jitter", "solar", "thermal")
SEED_COUNT = 20
RUN_SAMPLES = 200

EXPECTED = {
    "nominal": "NOMINAL",
    "rw_saturation": "MOMENTUM_DUMP",
    "rw_jitter": "ISOLATE_RW",
    "solar": "LOAD_SHED",
    "thermal": "SAFE",
}

_SOLAR_STEPS = (0.22, 0.27, 0.18, 0.25, 0.29, 0.16, 0.21, 0.24)


def _soc(family: str, seed: int, index: int, onset: int) -> float:
    if family == "nominal":
        return 0.8 - seed * 0.001
    if family != "solar" or index < onset:
        return 0.8
    return _SOLAR_STEPS[(index + seed) % len(_SOLAR_STEPS)]


def _temp(family: str, seed: int, index: int, onset: int) -> float:
    if family == "nominal":
        return 20.0 + seed * 0.05
    if family == "thermal" and index >= onset:
        return 60.0 + float(seed % 15)
    return 20.0


def _wheels(family: str, seed: int, index: int, onset: int) -> list[float]:
    wheels = [0.0, 0.0, 0.0, 0.0]
    wheel = seed % WHEEL_COUNT
    if family == "nominal":
        for index_w in range(WHEEL_COUNT):
            wheels[index_w] = ((index + seed + index_w) % 5 - 2) * 0.005
        return wheels
    if family == "rw_saturation" and index >= onset:
        wheels[wheel] = (0.91 + (seed % SEED_COUNT) * 0.004) * WHEEL_MAX_RAD_S
        return wheels
    if family == "rw_jitter":
        for index_w in range(WHEEL_COUNT):
            wheels[index_w] = ((index + seed + index_w) % 5 - 2) * 0.01
        if index >= onset:
            magnitude = 3.0 * (1.0 + seed * 0.01)
            sign = 1.0 if ((index - onset) % 2) == 0 else -1.0
            wheels[wheel] = sign * magnitude
        return wheels
    return wheels


def _solar_scale(family: str, index: int, onset: int) -> float:
    if family == "solar" and index >= onset:
        return 0.25
    return 1.0


def run_one(family: str, seed: int) -> dict:
    onset = seed
    wheel = seed % WHEEL_COUNT
    monitor = BusMonitor()
    manager = FdirManager(monitor)
    saw_safe = False
    last_mode = manager.mode
    last_wheel = manager.wheel_index
    for index in range(RUN_SAMPLES):
        sample = sensor_sample(
            index * STEP_NS,
            soc=_soc(family, seed, index, onset),
            solar_scale=_solar_scale(family, index, onset),
            wheel_speed_rad_s=_wheels(family, seed, index, onset),
            temp_c=_temp(family, seed, index, onset),
            heater_on=family == "thermal",
        )
        record = monitor.ingest(sample)
        decision = manager.tick(record, None, True)
        last_mode = decision.mode
        last_wheel = decision.wheel_index
        if decision.mode == "SAFE":
            saw_safe = True
    expected = EXPECTED[family]
    if family == "nominal":
        passed = last_mode == "NOMINAL" and not saw_safe
    elif family == "rw_jitter":
        passed = last_mode == "ISOLATE_RW" and last_wheel == wheel
    elif family == "rw_saturation":
        passed = last_mode == "MOMENTUM_DUMP" and last_wheel == wheel
    else:
        passed = last_mode == expected
    return {
        "family": family,
        "seed": seed,
        "passed": passed,
        "mode": last_mode,
        "wheel_index": last_wheel,
        "expected": expected,
        "saw_safe": saw_safe,
    }


def run_all() -> dict:
    runs: list[dict] = []
    failures: list[dict] = []
    family_pass = {family: 0 for family in FAMILIES}
    for family in FAMILIES:
        for seed in range(SEED_COUNT):
            try:
                result = run_one(family, seed)
            except Exception as exc:  # one bad seed must not hide the rest
                result = {
                    "family": family,
                    "seed": seed,
                    "passed": False,
                    "mode": None,
                    "wheel_index": None,
                    "expected": EXPECTED[family],
                    "error": f"{type(exc).__name__}: {exc}",
                }
            runs.append(result)
            if result["passed"]:
                family_pass[family] += 1
            else:
                failures.append(result)
    pass_count = sum(family_pass.values())
    return {
        "pass_count": pass_count,
        "total": len(FAMILIES) * SEED_COUNT,
        "fail_count": len(failures),
        "families": {
            family: {"pass_count": family_pass[family], "total": SEED_COUNT}
            for family in FAMILIES
        },
        "failures": failures,
        "runs": runs,
    }


def main() -> int:
    summary = run_all()
    SUMMARY_PATH.parent.mkdir(parents=True, exist_ok=True)
    SUMMARY_PATH.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8", newline="\n")
    print(
        f"pass_count={summary['pass_count']} total={summary['total']} "
        f"fail_count={summary['fail_count']} summary={SUMMARY_PATH}"
    )
    for failure in summary["failures"]:
        print(
            "FAIL "
            f"family={failure['family']} seed={failure['seed']} "
            f"mode={failure.get('mode')} expected={failure.get('expected')} "
            f"wheel_index={failure.get('wheel_index')} error={failure.get('error')}"
        )
    return 0 if summary["fail_count"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
