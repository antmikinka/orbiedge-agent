"""CLI: --summary-file, --once, --live.

Stdout is one line: a proposal JSON object, or the exact line NO PROPOSAL.
--live opens 127.0.0.1:50001 only when a model proposal survived the mirror.
choose_default is not called. A missing model sends nothing.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from advisor.oga_probe import probe
from advisor.proposal import model_proposal
from advisor.uplink import UplinkMirror, send_frame


def _load_summary(path: str) -> dict | None:
    try:
        raw = Path(path).read_text(encoding="utf-8")
        summary = json.loads(raw)
    except (OSError, json.JSONDecodeError):
        return None
    if not isinstance(summary, dict):
        return None
    return summary


def _model_text(info: dict) -> str | None:
    if not info.get("generated"):
        return None
    text = info.get("text")
    if not isinstance(text, str) or not text:
        return None
    return text


def run(summary: dict, *, live: bool, once: bool) -> str:
    info = probe()
    proposal = model_proposal(summary, _model_text(info))
    if proposal is None:
        return "NO PROPOSAL"
    if live:
        sim_time_ns = summary.get("sim_time_ns")
        command = {
            "opcode": "AGENT.PROPOSE",
            "action": proposal["action"],
            "wheel_index": proposal["wheel_index"],
            "confidence": proposal["confidence"],
            "sim_time_ns": sim_time_ns,
        }
        framed = UplinkMirror().frame(
            [command],
            current_sim_time_ns=sim_time_ns if isinstance(sim_time_ns, int) else 0,
            wall_time=time.monotonic(),
        )
        if framed is None:
            return "NO PROPOSAL"
        try:
            send_frame(framed)
        except OSError:
            return "NO PROPOSAL"
    if once or live:
        return json.dumps(proposal)
    return "NO PROPOSAL"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="advisor")
    parser.add_argument("--summary-file", required=True)
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--live", action="store_true")
    args = parser.parse_args(argv)
    summary = _load_summary(args.summary_file)
    if summary is None:
        print("NO PROPOSAL")
        return 0
    print(run(summary, live=args.live, once=args.once))
    return 0


if __name__ == "__main__":
    sys.exit(main())
