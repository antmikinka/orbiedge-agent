# Disk probe

Date: 2026-10-17.

Question: Is Llama 3.2 1B hybrid OGA installed on this Windows host?

Answer: No. Ryzen AI Software 1.8.0 is installed, and `onnxruntime_genai` imports, but no Llama 3.2 1B hybrid OGA model directory is on disk. Nothing was downloaded. RSS and TTFT were not measured and are not claimed.

`oga_client.py` therefore has no discovered model directory. With no `--model-dir`, it must log `model not loaded; sending nothing` and exit 0 without opening a socket.

## Python

`python` is `C:\Users\antmi\miniforge3\python.exe`. There is no `py` launcher.

Executed:

```
C:\Users\antmi\miniforge3\python.exe -c "import sys; print(sys.version); print(sys.executable)"
```

Output:

```
3.12.11 | packaged by conda-forge | (main, Jun  4 2025, 14:29:09) [MSC v.1943 64 bit (AMD64)]
C:\Users\antmi\miniforge3\python.exe
```

## pip

`python -m pip show onnxruntime-genai` exited 1:

```
WARNING: Package(s) not found: onnxruntime-genai
```

`python -m pip show onnxruntime-genai-directml-ryzenai`:

```
Name: onnxruntime-genai-directml-ryzenai
Version: 0.9.2.1
Author: Advanced Micro Devices, Inc.
Location: C:\Users\antmi\miniforge3\Lib\site-packages
Requires: numpy, onnxruntime-vitisai
```

`python -m pip show onnxruntime-genai-directml`:

```
Name: onnxruntime-genai-directml
Version: 0.10.0
Author: Microsoft Corporation
Location: C:\Users\antmi\miniforge3\Lib\site-packages
Requires: numpy, onnxruntime-directml
```

`pip list` also showed `onnxruntime 1.27.0`, `onnxruntime-directml 1.23.0`, and `onnxruntime-vitisai 1.23.0`. Both GenAI `RECORD` files list `onnxruntime_genai/__init__.py` (ryzenai record size 872, directml record size 792).

## Import

Executed:

```
C:\Users\antmi\miniforge3\python.exe -c "import onnxruntime_genai as og; print('file', og.__file__); print('version', getattr(og, '__version__', 'unknown'))"
```

Output:

```
file C:\Users\antmi\miniforge3\Lib\site-packages\onnxruntime_genai\__init__.py
version 0.9.2.1
```

That `__init__.py` contains `__version__ = "0.9.2.1"`, `__id__ = "onnxruntime-genai-directml"`, and `__rai_version__ = "1.6.0"`. The imported version matches the Ryzen package metadata (0.9.2.1), not the separate `onnxruntime-genai-directml` metadata version 0.10.0. The import was not followed by a model load.

## Ryzen AI Software

`C:\Program Files\RyzenAI` exists. Its children are `1.5.1`, `1.6.0`, `1.7.0`, and `1.8.0`.

`C:\Program Files\RyzenAI\1.8.0` contains `LLM`, `onnxruntime`, `onnxruntime_genai`, `quicktest`, and the uninstalled wheel `onnxruntime_genai_directml_ryzenai-0.14.0-py3-none-win_amd64.whl`. The imported module is 0.9.2.1, not that 0.14.0 wheel.

`C:\Program Files\RyzenAI\1.8.0\LLM` contains only `example`, `include`, and `lib`. A depth-3 name search found no Llama directory under `C:\Program Files\RyzenAI`, `C:\Program Files\AMD`, `C:\Program Files\AMD RyzenAI`, or `C:\Program Files (x86)\AMD`.

## Llama 3.2 1B hybrid OGA

Not installed.

- `Test-Path` of `C:\Users\antmi\.cache\huggingface\hub\models--amd--Llama-3.2-1B-Instruct-awq-g128-int4-asym-fp16-onnx-hybrid` was False.
- `Test-Path` of the historical file `...\snapshots\0bad422ccc41500719ba7904200fbd000b56332e\Llama-3.2-1B_jit.onnx` was False.
- The Hugging Face hub directory listing did not contain a Llama 3.2 model. The only `genai_config.json` found under the searched Hugging Face, Lemonade, `models`, FLM, IRON, and Ryzen AI LLM roots was `models--amd--AMD-OLMo-1B-SFT-DPO-onnx-ryzenai-1.7-hybrid`, whose `genai_config.json` says `"type": "olmo"`. That is not Llama 3.2 1B and is not the advisor model.
- Two Lemonade build directories exist for `amd_Llama-3.2-1B-Instruct-awq-g128-int4-asym-fp16-onnx-hybrid` (timestamps `2026y_02m_04d_03h_56m_49s` and `2026y_02m_04d_04h_00m_02s`). Each has `build_status: error` in `state.yaml`, a zero-byte `.lemonadebuild`, and no ONNX weights. The checkpoint string in those files is `amd/Llama-3.2-1B-Instruct-awq-g128-int4-asym-fp16-onnx-hybrid`. The later `log_oga-load.txt` records a `RuntimeError` loading the snapshot path above. Those leftovers are not a model directory.
- `C:\Users\antmi\Documents\flm\models\Llama-3.2-1B-NPU2` is a FastFlowLM tree (`model.q4nx`, `*.xclbin`, `flm_version` `0.9.21`). It has no `genai_config.json`. It is not an OGA model and is not used.
- `C:\Users\antmi\models` contains `embeddinggemma-300M-BF16` and `qwen3.8-27b` only.
- Olive recipe folders and `IRON\iron\models\llama32` are source or recipes, not OGA weights.
- Ollama manifests under `C:\Users\antmi\.ollama\models\manifests\registry.ollama.ai` were `library\embeddinggemma\latest` and `library\gpt-oss\20b`. No Llama manifest.

No RSS or TTFT figure was read from a run, because no OGA model was loaded.
