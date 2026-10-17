"""Optional ONNX Runtime GenAI advisor. Not the mode authority.

If onnxruntime_genai cannot be imported, or no model directory is passed and
the disk probe found none, write one log line and exit 0 without opening a
socket. A generation exception or a failed parse also sends nothing.

This process connects to 127.0.0.1:50001 only, and only after
parse_model_output accepts the model text. It does not bind.
"""

import argparse
import json
import sys
import time
from pathlib import Path

from proposal import build_propose_line, parse_model_output, predicate_prompt

HOST = "127.0.0.1"
ALLOWED_PORT = 50001
LOG_PATH = Path(__file__).resolve().parent / "advisor.log"
NOT_LOADED_LINE = "model not loaded; sending nothing"

# PROBE.md: no Llama 3.2 1B hybrid OGA directory was found.
# Do not substitute the FLM .q4nx tree or the OLMo hybrid snapshot.
DISCOVERED_MODEL_DIR = None


def write_log(text, overwrite=False):
    LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
    mode = "w" if overwrite else "a"
    line = text if text.endswith("\n") else text + "\n"
    with LOG_PATH.open(mode, encoding="utf-8", newline="\n") as handle:
        handle.write(line)


def working_set_bytes():
    """Process working set from the OS, or None if it cannot be read."""
    try:
        import ctypes
        from ctypes import wintypes

        class PROCESS_MEMORY_COUNTERS(ctypes.Structure):
            _fields_ = [
                ("cb", wintypes.DWORD),
                ("PageFaultCount", wintypes.DWORD),
                ("PeakWorkingSetSize", ctypes.c_size_t),
                ("WorkingSetSize", ctypes.c_size_t),
                ("QuotaPeakPagedPoolUsage", ctypes.c_size_t),
                ("QuotaPagedPoolUsage", ctypes.c_size_t),
                ("QuotaPeakNonPagedPoolUsage", ctypes.c_size_t),
                ("QuotaNonPagedPoolUsage", ctypes.c_size_t),
                ("PagefileUsage", ctypes.c_size_t),
                ("PeakPagefileUsage", ctypes.c_size_t),
            ]

        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        psapi = ctypes.WinDLL("psapi", use_last_error=True)
        get_current_process = kernel32.GetCurrentProcess
        get_current_process.restype = ctypes.c_void_p
        get_process_memory_info = psapi.GetProcessMemoryInfo
        get_process_memory_info.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(PROCESS_MEMORY_COUNTERS),
            wintypes.DWORD,
        ]
        get_process_memory_info.restype = wintypes.BOOL
        counters = PROCESS_MEMORY_COUNTERS()
        counters.cb = ctypes.sizeof(PROCESS_MEMORY_COUNTERS)
        ok = get_process_memory_info(
            get_current_process(),
            ctypes.byref(counters),
            counters.cb,
        )
        if not ok:
            return None
        return int(counters.WorkingSetSize)
    except (OSError, AttributeError, ValueError):
        return None


def resolve_model_dir(explicit):
    if explicit:
        path = Path(explicit)
        if path.is_dir():
            return path
        return None
    if not DISCOVERED_MODEL_DIR:
        return None
    path = Path(DISCOVERED_MODEL_DIR)
    if path.is_dir():
        return path
    return None


def load_state(state_file):
    if state_file:
        raw = Path(state_file).read_text(encoding="utf-8")
    else:
        raw = sys.stdin.read()
    state = json.loads(raw)
    if not isinstance(state, dict):
        raise ValueError("state must be a JSON object")
    sim_time_ns = state["sim_time_ns"]
    if isinstance(sim_time_ns, bool) or not isinstance(sim_time_ns, int):
        raise ValueError("sim_time_ns must be an int")
    return state


def generate_text(model_dir, prompt):
    import onnxruntime_genai as og

    started = time.perf_counter()
    model = og.Model(str(model_dir))
    tokenizer = og.Tokenizer(model)
    stream = tokenizer.create_stream()
    input_tokens = tokenizer.encode(prompt)
    params = og.GeneratorParams(model)
    params.set_search_options(
        max_length=int(len(input_tokens) + 64),
        do_sample=False,
    )
    generator = og.Generator(model, params)
    generator.append_tokens(input_tokens)
    parts = []
    while not generator.is_done():
        generator.generate_next_token()
        parts.append(stream.decode(generator.get_next_tokens()[0]))
    elapsed = time.perf_counter() - started
    working_set = working_set_bytes()
    working_set_text = "unavailable" if working_set is None else str(working_set)
    write_log(
        "generation_wall_clock_s={:.6f} working_set_bytes={}".format(
            elapsed,
            working_set_text,
        )
    )
    return "".join(parts)


def send_proposal(line):
    import socket

    with socket.create_connection((HOST, ALLOWED_PORT), timeout=5) as sock:
        sock.sendall(line.encode("utf-8"))


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Optional OGA advisor. Not the mode authority."
    )
    parser.add_argument("--model-dir", default=None)
    parser.add_argument("--state-file", default=None)
    args = parser.parse_args(argv)

    try:
        import onnxruntime_genai  # noqa: F401
    except ImportError:
        write_log(NOT_LOADED_LINE, overwrite=True)
        return 0

    model_dir = resolve_model_dir(args.model_dir)
    if model_dir is None:
        write_log(NOT_LOADED_LINE, overwrite=True)
        return 0

    try:
        state = load_state(args.state_file)
        text = generate_text(model_dir, predicate_prompt(state))
        parsed = parse_model_output(text)
        if parsed is None:
            write_log("parser failed; sending nothing")
            return 0
        line = build_propose_line(
            parsed["action"],
            parsed["wheel_index"],
            parsed["confidence"],
            state["sim_time_ns"],
        )
        send_proposal(line)
    except Exception:
        write_log("generation failed; sending nothing")
        return 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
