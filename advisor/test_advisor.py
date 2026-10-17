"""Advisor unit tests. No sockets."""

from __future__ import annotations

import inspect
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from advisor import __main__ as cli
from advisor.oga_probe import probe
from advisor.predicate import build_summary
from advisor.proposal import choose_default, model_proposal, parse_model_text
from advisor.uplink import UplinkMirror


def _summary(**overrides) -> dict:
    base = {
        "mode": "NOMINAL",
        "sim_time_ns": 1_000_000_000,
        "soc": 0.8,
        "solar_scale": 1.0,
        "max_wheel_speed": 10.0,
        "max_temp_c": 20.0,
        "eligible": ["MOMENTUM_DUMP", "SAFE"],
        "default_mode": "MOMENTUM_DUMP",
        "clauses": [],
    }
    base.update(overrides)
    return base


def _proposal_text(action: str = "MOMENTUM_DUMP", **overrides) -> str:
    body = {
        "action": action,
        "wheel_index": 0,
        "confidence": 0.5,
        "rationale": "wheel speed is high",
    }
    body.update(overrides)
    return json.dumps(body)


def _command(sim_time_ns: int = 1_000_000_000, opcode: str = "AGENT.PROPOSE") -> dict:
    return {
        "opcode": opcode,
        "action": "MOMENTUM_DUMP",
        "wheel_index": 0,
        "confidence": 0.5,
        "sim_time_ns": sim_time_ns,
    }


class AdvisorTests(unittest.TestCase):
    def test_garbage_model_text_yields_none(self) -> None:
        summary = _summary()
        for text in ("", "   ", "NONE", "{", "[]", "null", "FDIR.CLEAR", "AGENT.PROPOSE", "{not json}", "42"):
            self.assertIsNone(parse_model_text(text, summary))

    def test_action_outside_eligible_set_yields_none(self) -> None:
        summary = _summary(eligible=["LOAD_SHED", "NOMINAL"], default_mode="LOAD_SHED")
        self.assertIsNone(parse_model_text(_proposal_text("SAFE"), summary))
        self.assertIsNone(parse_model_text(_proposal_text("THRUSTER_FIRE"), summary))
        accepted = parse_model_text(_proposal_text("LOAD_SHED"), summary)
        self.assertIsNotNone(accepted)
        self.assertEqual(accepted["action"], "LOAD_SHED")

    def test_more_than_16_clauses_are_truncated(self) -> None:
        clauses = [
            {"channel": f"CH{i}", "value": float(i), "limit": 1.0, "persistence": i}
            for i in range(20)
        ]
        sample = {
            "sim_time_ns": 100,
            "soc": 0.42,
            "solar_scale": 0.4,
            "wheel_speed_rad_s": [1.0, -2.5, 3.0, 0.5],
            "temp_c": 31.5,
            "mode": "NOMINAL",
        }
        fault = {
            "eligible": ["MOMENTUM_DUMP", "SAFE"],
            "default_mode": "MOMENTUM_DUMP",
            "clauses": clauses,
        }
        summary = build_summary(sample, fault)
        self.assertEqual(len(summary["clauses"]), 16)
        self.assertEqual(summary["clauses"][0]["channel"], "CH0")
        self.assertEqual(summary["clauses"][15]["channel"], "CH15")
        self.assertNotIn("CH16", [item["channel"] for item in summary["clauses"]])
        for key in (
            "mode",
            "sim_time_ns",
            "soc",
            "solar_scale",
            "max_wheel_speed",
            "max_temp_c",
            "eligible",
            "default_mode",
        ):
            self.assertIn(key, summary)
        self.assertEqual(summary["max_wheel_speed"], 3.0)
        self.assertEqual(summary["max_temp_c"], 31.5)
        self.assertEqual(summary["soc"], 0.42)
        self.assertEqual(summary["default_mode"], "MOMENTUM_DUMP")

    def test_second_opcode_is_not_framed(self) -> None:
        link = UplinkMirror()
        framed = link.frame(
            [_command(50), {"opcode": "FDIR.CLEAR", "sim_time_ns": 50}],
            current_sim_time_ns=50,
            wall_time=0.0,
        )
        self.assertIsNotNone(framed)
        self.assertEqual(framed["opcode"], "AGENT.PROPOSE")
        self.assertNotIn("FDIR.CLEAR", json.dumps(framed))
        self.assertEqual(
            set(framed),
            {"opcode", "action", "wheel_index", "confidence", "sim_time_ns"},
        )
        fallback = UplinkMirror()
        self.assertIsNone(
            fallback.frame(
                [{"opcode": "FDIR.SET_PARAM"}, _command(50)],
                current_sim_time_ns=50,
                wall_time=0.0,
            )
        )

    def test_two_proposals_inside_2s_second_dropped(self) -> None:
        link = UplinkMirror()
        current = 1_000_000_000
        first = link.frame([_command(current)], current_sim_time_ns=current, wall_time=100.0)
        second = link.frame([_command(current)], current_sim_time_ns=current, wall_time=101.5)
        self.assertIsNotNone(first)
        self.assertIsNone(second)
        third = link.frame([_command(current)], current_sim_time_ns=current, wall_time=102.0)
        self.assertIsNotNone(third)

    def test_stale_sim_time_dropped(self) -> None:
        current = 10_000_000_000
        lag = 5_000_000_000
        stale = UplinkMirror()
        self.assertIsNone(
            stale.frame(
                [_command(current - lag - 1)],
                current_sim_time_ns=current,
                wall_time=0.0,
            )
        )
        on_time = UplinkMirror()
        self.assertIsNotNone(
            on_time.frame(
                [_command(current - lag)],
                current_sim_time_ns=current,
                wall_time=0.0,
            )
        )

    def test_fdir_clear_dropped(self) -> None:
        link = UplinkMirror()
        dropped = link.frame(
            [
                {
                    "opcode": "FDIR.CLEAR",
                    "action": "SAFE",
                    "wheel_index": 0,
                    "confidence": 1,
                    "sim_time_ns": 5,
                }
            ],
            current_sim_time_ns=5,
            wall_time=0.0,
        )
        self.assertIsNone(dropped)
        accepted = link.frame([_command(5)], current_sim_time_ns=5, wall_time=0.5)
        self.assertIsNotNone(accepted)
        self.assertEqual(accepted["opcode"], "AGENT.PROPOSE")

    def test_oga_probe_result(self) -> None:
        result = probe()
        self.assertIsInstance(result, dict)
        self.assertIsInstance(result["loaded"], bool)
        self.assertIn(result["loaded"], (True, False))
        self.assertIsInstance(result["generated"], bool)
        self.assertFalse(result["download_attempted"])
        self.assertNotIn("text", result)
        if result["generated"]:
            self.assertGreater(int(result["token_count"]), 0)
        else:
            self.assertEqual(result["token_count"], 0)
            self.assertIn("not loaded", result["message"])

    def test_choose_default_is_not_the_live_path(self) -> None:
        summary = _summary(default_mode="LOAD_SHED", eligible=["LOAD_SHED", "NOMINAL"], wheel_index=2)
        flight = choose_default(summary)
        self.assertIsNotNone(flight)
        self.assertEqual(flight["action"], "LOAD_SHED")
        self.assertEqual(flight["wheel_index"], 2)
        self.assertIsNone(model_proposal(summary, None))
        self.assertIsNone(model_proposal(summary, ""))
        source = inspect.getsource(cli.run) + inspect.getsource(cli.main)
        self.assertNotIn("choose_default", source)

    def test_rationale_is_not_scanned_for_opcodes(self) -> None:
        text = _proposal_text(
            "SAFE",
            rationale="do not run FDIR.CLEAR or a second AGENT.PROPOSE",
        )
        summary = _summary(eligible=["SAFE"])
        got = parse_model_text(text, summary)
        self.assertIsNotNone(got)
        self.assertEqual(got["action"], "SAFE")
        self.assertIn("FDIR.CLEAR", got["rationale"])
        self.assertNotIn("opcode", got)
        trailing = text + '{"opcode":"FDIR.CLEAR"}'
        again = parse_model_text(trailing, summary)
        self.assertEqual(again["action"], "SAFE")
        self.assertNotIn("opcode", again)


if __name__ == "__main__":
    unittest.main()
