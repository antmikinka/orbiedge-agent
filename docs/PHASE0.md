# Phase 0 probe

Recorded 2026-10-17 from this Windows host. Items below are what was executed. A missing measurement is not a result.

## Pins

| Dependency | Pin in this tree | Note |
| --- | --- | --- |
| nasa/fprime | submodule `third_party/fprime`, branch `devel` | Present. `Svc/Time` is not a directory. The closest time hook is `Svc.OsTime`: sync input `setEpoch` and command `SetCurrentTime(seconds_now: U32)`. |
| nasa/fprime-gds | not a submodule | Not pinned. |
| AVSLab/basilisk | submodule `third_party/basilisk`, branch `develop` | `examples/scenarioAttitudeFeedbackRWPower.py` is in that checkout. The architecture asks for a release pin. This tree is still on `develop`. The scenario that ran used the installed Basilisk 2.12.0 wheel, which is older than that checkout. |

## NPU

Ryzen AI Software 1.8.0 is installed at `C:\Program Files\RyzenAI\1.8.0`. Older trees 1.5.1, 1.6.0, and 1.7.0 are also present.

`onnxruntime_genai` 0.9.2.1 imports from `C:\Users\antmi\miniforge3\Lib\site-packages\onnxruntime_genai\__init__.py`. That is not the uninstalled 1.8 wheel `onnxruntime_genai_directml_ryzenai-0.14.0`.

The Llama 3.2 1B hybrid weights are not on disk. Nothing was downloaded. No model was loaded. No tokens were generated. RSS and TTFT were not measured. A cached `AMD-OLMo-1B` hybrid snapshot is a different model and was not run. `advisor/PROBE.md` has the file-by-file search.

## Localhost ports

Default WSL distro: `docker-desktop`, stopped. Probe distro: `UbuntuOEM`, WSL2, Ubuntu 24.04.3 LTS.

Ubuntu bound `127.0.0.1` on 50000, 50001, 5600, and 5601. A Windows `TcpClient` connected to each of those four ports during the bind. This did not start F´, GDS, or Basilisk.

## Not measured

Windows power-counter tool name. Advisor RSS. Advisor time to first token. An F´ deployment boot. The 100-run Basilisk matrix.
