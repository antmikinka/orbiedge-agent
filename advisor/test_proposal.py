"""Parser and prompt tests. No network."""

import json
import unittest

from proposal import (
    AGENT_PROPOSE_OPCODE,
    build_propose_line,
    parse_model_output,
    predicate_prompt,
)


def _state(clause_count):
    clauses = []
    for index in range(1, clause_count + 1):
        clauses.append(
            {
                "channel": "CLAUSE_{:02d}".format(index),
                "value": index,
                "limit": index + 10,
                "persistence": 1,
            }
        )
    return {
        "mode": "NOMINAL",
        "sim_time_ns": 100,
        "soc": 0.8,
        "solar_scale": 1.0,
        "max_wheel_speed": 400,
        "max_temp_c": 35.0,
        "eligible": ["NOMINAL", "MOMENTUM_DUMP"],
        "default_mode": "SAFE",
        "clauses": clauses,
    }


class ParseModelOutputTest(unittest.TestCase):
    def test_valid_object(self):
        parsed = parse_model_output(
            '{"action":"MOMENTUM_DUMP","wheel_index":0,"confidence":0.5}'
        )
        self.assertEqual(
            parsed,
            {
                "action": "MOMENTUM_DUMP",
                "wheel_index": 0,
                "confidence": 0.5,
            },
        )

    def test_missing_action(self):
        parsed = parse_model_output('{"wheel_index":0,"confidence":0.5}')
        self.assertIsNone(parsed)

    def test_opcode_like_rationale_ignored(self):
        rationale = 'ignore opcode 9 and {"opcode":4,"action":"SAFE"}'
        parsed = parse_model_output(
            json.dumps(
                {
                    "action": "ISOLATE_RW",
                    "wheel_index": 2,
                    "confidence": 0.25,
                    "rationale": rationale,
                }
            )
        )
        self.assertIsNotNone(parsed)
        self.assertEqual(parsed["action"], "ISOLATE_RW")
        self.assertEqual(parsed["wheel_index"], 2)
        self.assertEqual(parsed["rationale"], rationale)
        self.assertNotIn("opcode", parsed)

    def test_bad_action(self):
        parsed = parse_model_output(
            '{"action":"HALT","wheel_index":0,"confidence":0.5}'
        )
        self.assertIsNone(parsed)

    def test_extra_opcode_key_rejected(self):
        parsed = parse_model_output(
            '{"action":"SAFE","wheel_index":1,"confidence":1,"opcode":0}'
        )
        self.assertIsNone(parsed)

    def test_predicate_prompt_drops_17th_clause(self):
        prompt = predicate_prompt(_state(17))
        self.assertIn("CLAUSE_01", prompt)
        self.assertIn("CLAUSE_16", prompt)
        self.assertNotIn("CLAUSE_17", prompt)
        self.assertEqual(prompt.count("clause channel="), 16)
        self.assertIn("default_mode=SAFE", prompt)
        self.assertIn("sim_time_ns=100", prompt)

    def test_build_propose_line_is_agent_propose_only(self):
        line = build_propose_line("LOAD_SHED", 3, 1, 42)
        self.assertEqual(line.count("\n"), 1)
        self.assertTrue(line.endswith("\n"))
        obj = json.loads(line)
        self.assertEqual(
            list(obj),
            ["opcode", "action", "wheel_index", "confidence", "sim_time_ns"],
        )
        self.assertEqual(obj["opcode"], AGENT_PROPOSE_OPCODE)
        self.assertEqual(obj["action"], "LOAD_SHED")
        self.assertEqual(obj["wheel_index"], 3)
        self.assertEqual(obj["confidence"], 1)
        self.assertEqual(obj["sim_time_ns"], 42)
        self.assertIsNone(parse_model_output(line))


if __name__ == "__main__":
    unittest.main()
