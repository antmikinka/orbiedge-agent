"""Flight-rule unit tests. Standard library only.

Run from the repo root:

    python -m unittest tests.test_fdir_rules
"""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sim.canned.thermal_critical import run  # noqa: E402
from sim.fdir.rules import (  # noqa: E402
    PROPOSAL_STALE_NS,
    STEP_NS,
    TEMP_CRIT_SAMPLES,
    WHEEL_MAX_RAD_S,
    BusMonitor,
    FaultRecord,
    FdirManager,
    sensor_sample,
    uplink_filter,
)
from sim.fdir.wire import format_mode_line, parse_mode_line, parse_sensor_line  # noqa: E402


def _pair() -> tuple[BusMonitor, FdirManager]:
    monitor = BusMonitor()
    return monitor, FdirManager(monitor)


def _drive(monitor: BusMonitor, manager: FdirManager, samples: list[dict], proposals=None):
    proposals = proposals or {}
    decisions = []
    for index, sample in enumerate(samples):
        record = monitor.ingest(sample)
        decision = manager.tick(record, proposals.get(index), True)
        decisions.append(decision)
    return decisions


class FdirRulesTest(unittest.TestCase):
    def test_thermal_safe_without_proposal(self) -> None:
        timeline = run()
        self.assertEqual(timeline[0], "NOMINAL")
        self.assertIn("SAFE", timeline)
        self.assertEqual(timeline[-1], "SAFE")
        self.assertGreaterEqual(timeline.count("SAFE"), 1)

    def test_nominal_stays_nominal(self) -> None:
        monitor, manager = _pair()
        samples = [sensor_sample(i * STEP_NS, soc=0.8, temp_c=20.0) for i in range(40)]
        decisions = _drive(monitor, manager, samples)
        self.assertTrue(all(decision.mode == "NOMINAL" for decision in decisions))
        self.assertEqual(manager.mode, "NOMINAL")

    def test_soc_critical_becomes_safe(self) -> None:
        monitor, manager = _pair()
        samples = [sensor_sample(i * STEP_NS, soc=0.10) for i in range(60)]
        decisions = _drive(monitor, manager, samples)
        self.assertEqual(decisions[-1].mode, "SAFE")
        self.assertEqual(manager.mode, "SAFE")

    def test_soc_warning_becomes_load_shed(self) -> None:
        monitor, manager = _pair()
        samples = [sensor_sample(i * STEP_NS, soc=0.25) for i in range(100)]
        decisions = _drive(monitor, manager, samples)
        self.assertEqual(decisions[-1].mode, "LOAD_SHED")
        self.assertNotEqual(manager.mode, "SAFE")

    def test_overspeed_becomes_momentum_dump(self) -> None:
        monitor, manager = _pair()
        fast = 0.95 * WHEEL_MAX_RAD_S
        samples = []
        for index in range(50):
            samples.append(
                sensor_sample(
                    index * STEP_NS,
                    wheel_speed_rad_s=(0.0, 0.0, 0.0, fast),
                )
            )
        decisions = _drive(monitor, manager, samples)
        self.assertEqual(decisions[-1].mode, "MOMENTUM_DUMP")
        self.assertEqual(decisions[-1].wheel_index, 3)

    def test_jitter_proposal_isolate_is_agree(self) -> None:
        monitor, manager = _pair()
        agreed = None
        for index in range(40):
            wheels = [0.0, 0.0, 0.0, 0.0]
            wheels[2] = 3.0 if index % 2 == 0 else -3.0
            record = monitor.ingest(sensor_sample(index * STEP_NS, wheel_speed_rad_s=wheels))
            proposal = None
            if record.severity == "WARNING" and "ISOLATE_RW" in record.eligible and agreed is None:
                proposal = {
                    "action": "ISOLATE_RW",
                    "wheel_index": 2,
                    "confidence": 0.95,
                    "sim_time_ns": record.sim_time_ns,
                }
            decision = manager.tick(record, proposal, True)
            if proposal is not None:
                agreed = decision
                break
        self.assertIsNotNone(agreed)
        assert agreed is not None
        self.assertEqual(agreed.disposition, "AGREE")
        self.assertEqual(agreed.mode, "ISOLATE_RW")
        self.assertEqual(agreed.wheel_index, 2)

    def test_load_shed_proposal_during_overspeed_is_reject(self) -> None:
        monitor, manager = _pair()
        fast = 0.95 * WHEEL_MAX_RAD_S
        rejected = None
        for index in range(30):
            record = monitor.ingest(
                sensor_sample(index * STEP_NS, wheel_speed_rad_s=(fast, 0.0, 0.0, 0.0))
            )
            proposal = None
            if record.severity == "WARNING" and record.default_mode == "MOMENTUM_DUMP":
                proposal = {
                    "action": "LOAD_SHED",
                    "wheel_index": 0,
                    "confidence": 0.8,
                    "sim_time_ns": record.sim_time_ns,
                }
            decision = manager.tick(record, proposal, True)
            if proposal is not None:
                rejected = decision
                break
        self.assertIsNotNone(rejected)
        assert rejected is not None
        self.assertEqual(rejected.disposition, "REJECT")
        self.assertEqual(rejected.mode, "NOMINAL")

    def test_stale_proposal_is_late(self) -> None:
        monitor, manager = _pair()
        fast = 0.95 * WHEEL_MAX_RAD_S
        late = None
        for index in range(30):
            record = monitor.ingest(
                sensor_sample(index * STEP_NS, wheel_speed_rad_s=(0.0, fast, 0.0, 0.0))
            )
            proposal = None
            if record.severity == "WARNING" and record.default_mode == "MOMENTUM_DUMP":
                proposal = {
                    "action": "MOMENTUM_DUMP",
                    "wheel_index": 1,
                    "confidence": 1,
                    "sim_time_ns": record.sim_time_ns - PROPOSAL_STALE_NS - 1,
                }
            decision = manager.tick(record, proposal, True)
            if proposal is not None:
                late = decision
                break
        self.assertIsNotNone(late)
        assert late is not None
        self.assertEqual(late.disposition, "LATE")
        self.assertEqual(late.mode, "NOMINAL")

    def test_uplink_drops_non_propose_and_second_inside_two_seconds(self) -> None:
        now_sim = 10_000_000_000
        proposal = {
            "action": "LOAD_SHED",
            "wheel_index": 0,
            "confidence": 0.5,
            "sim_time_ns": now_sim,
        }
        dropped, reason = uplink_filter("FDIR.CLEAR", proposal, now_sim, 10.0, None)
        self.assertFalse(dropped)
        self.assertEqual(reason, "dropped_clear")
        dropped, reason = uplink_filter("PING", proposal, now_sim, 10.0, None)
        self.assertFalse(dropped)
        self.assertEqual(reason, "dropped_opcode")
        accepted, reason = uplink_filter("AGENT.PROPOSE", proposal, now_sim, 10.0, None)
        self.assertTrue(accepted)
        self.assertEqual(reason, "ok")
        accepted, reason = uplink_filter("AGENT.PROPOSE", proposal, now_sim, 11.9, 10.0)
        self.assertFalse(accepted)
        self.assertEqual(reason, "rate_limit")
        stale = dict(proposal)
        stale["sim_time_ns"] = now_sim - PROPOSAL_STALE_NS - 1
        accepted, reason = uplink_filter("AGENT.PROPOSE", stale, now_sim, 30.0, None)
        self.assertFalse(accepted)
        self.assertEqual(reason, "stale")

    def test_invalid_sensors_hold_mode(self) -> None:
        monitor, manager = _pair()
        samples = [sensor_sample(i * STEP_NS, soc=0.25) for i in range(100)]
        decisions = _drive(monitor, manager, samples)
        self.assertEqual(decisions[-1].mode, "LOAD_SHED")
        record = FaultRecord(
            sim_time_ns=100 * STEP_NS,
            eligible=("SAFE",),
            default_mode="SAFE",
            severity="CRITICAL",
            wheel_index=-1,
            failed=[("temp_c", 70.0, 60.0)],
            event="TEMP_CRIT",
            temp_warn=False,
        )
        proposal = {
            "action": "SAFE",
            "wheel_index": 0,
            "confidence": 1,
            "sim_time_ns": record.sim_time_ns,
        }
        decision = manager.tick(record, proposal, False)
        self.assertEqual(decision.disposition, "HOLD_INVALID")
        self.assertEqual(decision.mode, "LOAD_SHED")
        self.assertEqual(manager.mode, "LOAD_SHED")

    def test_safe_stays_sticky_until_clear(self) -> None:
        monitor, manager = _pair()
        index = 0
        last = None
        for _ in range(TEMP_CRIT_SAMPLES):
            record = monitor.ingest(sensor_sample(index * STEP_NS, temp_c=70.0, heater_on=True))
            last = manager.tick(record, None, True)
            index += 1
        assert last is not None
        self.assertEqual(last.mode, "SAFE")
        for _ in range(700):
            record = monitor.ingest(sensor_sample(index * STEP_NS, temp_c=20.0, soc=0.8))
            last = manager.tick(record, None, True)
            index += 1
            self.assertEqual(last.mode, "SAFE")
        cleared = manager.clear()
        self.assertEqual(cleared.mode, "NOMINAL")
        self.assertEqual(manager.mode, "NOMINAL")
        record = monitor.ingest(sensor_sample(index * STEP_NS, temp_c=20.0, soc=0.8))
        last = manager.tick(record, None, True)
        self.assertEqual(last.mode, "NOMINAL")

    def test_mode_line_rejects_unknown_mode(self) -> None:
        line = format_mode_line({"sim_time_ns": 0, "mode": "NOMINAL", "wheel_index": -1})
        self.assertEqual(parse_mode_line(line)["mode"], "NOMINAL")
        with self.assertRaises(ValueError):
            format_mode_line({"sim_time_ns": 0, "mode": "HOLD", "wheel_index": 0})
        sensor = sensor_sample(STEP_NS, soc=0.5, temp_c=21.0, heater_on=False)
        parsed = parse_sensor_line(json.dumps(sensor))
        self.assertEqual(parsed["wheel_speed_rad_s"], [0.0, 0.0, 0.0, 0.0])
        self.assertEqual(len(parsed), 6)


if __name__ == "__main__":
    unittest.main()
