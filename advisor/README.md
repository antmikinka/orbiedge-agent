# OrbiEdge Windows advisor

FdirManager is the mode authority. This process is optional. It may only propose a mode. It does not select one.

Killing this process is the Phase 1 acceptance configuration. If it is killed, or if its parser fails, it sends nothing. FdirManager applies the table default.

The advisor may open `127.0.0.1:50001` only, as a TCP client. It must not bind. It must not connect to 50000, 5600, or 5601. The only bytes it may write are one `AGENT.PROPOSE` line produced by `build_propose_line`.

`scripted_client.py` is a test double. It is not the NPU model. If the proposal would fail `parse_model_output`, it sends nothing and exits 2. A refused port sends nothing and exits 3.

`oga_client.py` imports `onnxruntime_genai` only to run a local model directory. The disk probe did not find a Llama 3.2 1B hybrid OGA model. With no model directory, the process writes one line to `advisor.log` and exits 0 without opening a socket. A generation exception also sends nothing.

RSS and TTFT are not claimed. See `PROBE.md` for what was found on this host. No model was downloaded.
