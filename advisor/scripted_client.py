"""Scripted test double. This is not the NPU model.

TCP client to 127.0.0.1:50001 only. It never binds and never connects to
50000, 5600, or 5601. It writes one AGENT.PROPOSE line from
build_propose_line, then closes. If parse_model_output would reject the
proposal, it writes nothing and exits 2.
"""

import argparse
import json
import socket
import sys

from proposal import build_propose_line, parse_model_output

HOST = "127.0.0.1"
ALLOWED_PORT = 50001
REFUSED_PORTS = frozenset({50000, 5600, 5601})


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Scripted advisor test double. Not the NPU model."
    )
    parser.add_argument("--action", required=True)
    parser.add_argument("--wheel-index", type=int, required=True)
    parser.add_argument("--confidence", type=float, required=True)
    parser.add_argument("--sim-time-ns", type=int, required=True)
    parser.add_argument("--port", type=int, default=ALLOWED_PORT)
    args = parser.parse_args(argv)

    if args.port in REFUSED_PORTS or args.port != ALLOWED_PORT:
        print("refusing port; sending nothing", file=sys.stderr)
        return 3

    try:
        shaped = json.dumps(
            {
                "action": args.action,
                "wheel_index": args.wheel_index,
                "confidence": args.confidence,
            },
            separators=(",", ":"),
            allow_nan=False,
        )
    except (ValueError, TypeError):
        print("proposal rejected; sending nothing", file=sys.stderr)
        return 2
    if parse_model_output(shaped) is None:
        print("proposal rejected; sending nothing", file=sys.stderr)
        return 2

    try:
        line = build_propose_line(
            args.action,
            args.wheel_index,
            args.confidence,
            args.sim_time_ns,
        )
    except (ValueError, TypeError):
        print("proposal rejected; sending nothing", file=sys.stderr)
        return 2

    with socket.create_connection((HOST, ALLOWED_PORT), timeout=5) as sock:
        sock.sendall(line.encode("utf-8"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
