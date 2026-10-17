"""Pure proposal helpers. Standard library only.

FdirManager is the mode authority. These functions never open a socket.
The only opcode this process may emit is AGENT.PROPOSE.
"""

from __future__ import annotations

import json
import math

# The name is the contract until a generated F´ dictionary supplies the number.
# A literal 0 would be a different command and the uplink filter would drop it.
AGENT_PROPOSE_OPCODE = "AGENT.PROPOSE"

ACTIONS = ("NOMINAL", "ISOLATE_RW", "MOMENTUM_DUMP", "LOAD_SHED", "SAFE")

_MODEL_KEYS = frozenset({"action", "wheel_index", "confidence", "rationale"})


def parse_model_output(text):
    """Return a proposal dict, or None if text is not a valid model object.

    Accepts only a JSON object with action, wheel_index, confidence, and an
    optional rationale string. rationale is stored and never scanned.
    """
    if not isinstance(text, str):
        return None
    try:
        obj = json.loads(text)
    except (json.JSONDecodeError, ValueError, TypeError):
        return None
    if not isinstance(obj, dict):
        return None
    if not set(obj).issubset(_MODEL_KEYS):
        return None
    if "action" not in obj or "wheel_index" not in obj or "confidence" not in obj:
        return None
    action = obj["action"]
    if not isinstance(action, str) or action not in ACTIONS:
        return None
    wheel_index = obj["wheel_index"]
    if isinstance(wheel_index, bool) or not isinstance(wheel_index, int):
        return None
    if wheel_index < 0 or wheel_index > 3:
        return None
    confidence = obj["confidence"]
    if isinstance(confidence, bool) or not isinstance(confidence, (int, float)):
        return None
    confidence = float(confidence)
    if not math.isfinite(confidence) or confidence < 0.0 or confidence > 1.0:
        return None
    parsed = {
        "action": action,
        "wheel_index": wheel_index,
        "confidence": confidence,
    }
    if "rationale" in obj:
        rationale = obj["rationale"]
        if not isinstance(rationale, str):
            return None
        parsed["rationale"] = rationale
    return parsed


def build_propose_line(action, wheel_index, confidence, sim_time_ns):
    """Return the only JSON line a client may write, including the trailing newline.

    Raises ValueError when the model-shaped fields would not parse, or when
    sim_time_ns is not an int. Callers that catch this must write nothing.
    """
    shaped = json.dumps(
        {
            "action": action,
            "wheel_index": wheel_index,
            "confidence": confidence,
        },
        separators=(",", ":"),
        allow_nan=False,
    )
    if parse_model_output(shaped) is None:
        raise ValueError("model-shaped proposal rejected")
    if isinstance(sim_time_ns, bool) or not isinstance(sim_time_ns, int):
        raise ValueError("sim_time_ns must be an int")
    line = {
        "opcode": AGENT_PROPOSE_OPCODE,
        "action": action,
        "wheel_index": wheel_index,
        "confidence": confidence,
        "sim_time_ns": sim_time_ns,
    }
    return json.dumps(
        line,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ) + "\n"


def predicate_prompt(state):
    """Short prompt. At most the first 16 clauses are included."""
    clauses = state["clauses"][:16]
    lines = [
        "Optional advisor only. FdirManager is the mode authority.",
        "Return one JSON object and no other text.",
        "Keys: action, wheel_index, confidence, optional rationale.",
        "action must be NOMINAL, ISOLATE_RW, MOMENTUM_DUMP, LOAD_SHED, or SAFE.",
        "wheel_index is an integer 0..3. confidence is a number 0..1.",
        "Do not include an opcode. Silence lets FdirManager apply default_mode.",
        "mode={}".format(state["mode"]),
        "sim_time_ns={}".format(state["sim_time_ns"]),
        "soc={}".format(state["soc"]),
        "solar_scale={}".format(state["solar_scale"]),
        "max_wheel_speed={}".format(state["max_wheel_speed"]),
        "max_temp_c={}".format(state["max_temp_c"]),
        "eligible={}".format(json.dumps(state["eligible"])),
        "default_mode={}".format(state["default_mode"]),
    ]
    for clause in clauses:
        lines.append(
            "clause channel={channel} value={value} limit={limit} persistence={persistence}".format(
                channel=clause["channel"],
                value=clause["value"],
                limit=clause["limit"],
                persistence=clause["persistence"],
            )
        )
    return "\n".join(lines) + "\n"


def _first_object(text: object) -> dict | None:
    if not isinstance(text, str):
        return None
    start = text.find("{")
    if start < 0:
        return None
    try:
        obj, _end = json.JSONDecoder().raw_decode(text, start)
    except json.JSONDecodeError:
        return None
    if not isinstance(obj, dict):
        return None
    return obj


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


def parse_model_text(text: object, summary: object) -> dict | None:
    """Return one proposal, or None on parser failure.

    The rationale string is copied and is not searched for opcodes.
    Only the first JSON object is read. A later object is not a fallback.
    The action must be one of the five mode enums and a member of eligible.
    """
    obj = _first_object(text)
    if obj is None or not isinstance(summary, dict):
        return None
    eligible = summary.get("eligible")
    if not isinstance(eligible, list):
        return None
    action = obj.get("action")
    if not isinstance(action, str):
        return None
    action = action.strip()
    if action not in ACTIONS or action not in eligible:
        return None
    wheel_index = _wheel_index(obj.get("wheel_index"))
    confidence = _confidence(obj.get("confidence"))
    rationale = obj.get("rationale")
    if wheel_index is None or confidence is None or not isinstance(rationale, str):
        return None
    return {
        "action": action,
        "wheel_index": wheel_index,
        "confidence": confidence,
        "rationale": rationale,
    }


def model_proposal(summary: object, model_text: object) -> dict | None:
    """Live entry. No model text means no proposal. Does not call choose_default."""
    if not isinstance(model_text, str) or not model_text:
        return None
    return parse_model_text(model_text, summary)


def choose_default(summary: object) -> dict | None:
    """What F Prime applies when no proposal arrives. Tests only.

    Not a transmit path. Unknown default_mode values are not turned into opcodes.
    """
    if not isinstance(summary, dict):
        return None
    action = summary.get("default_mode")
    if not isinstance(action, str) or action not in ACTIONS:
        return None
    wheel_index = summary.get("wheel_index", 0)
    checked = _wheel_index(wheel_index)
    if checked is None:
        checked = 0
    return {
        "action": action,
        "wheel_index": checked,
        "confidence": 1.0,
        "rationale": "F Prime applies default_mode when no proposal arrives",
    }
