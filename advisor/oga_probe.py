"""Probe for a local OGA stack. Never download a model.

loaded is true only when weights were actually loaded into a session.
This probe does not construct a model and does not generate tokens.
"""

from __future__ import annotations

import importlib
import importlib.util
import os

_MODULES = ("onnxruntime_genai", "oga", "ryzenai")
_INSTALL_ENV = ("OGA_MODEL_PATH", "ORBIEDGE_OGA_MODEL", "RYZEN_AI_INSTALLATION_PATH")
_INSTALL_DIRS = (
    r"C:\Program Files\RyzenAI\1.8.0",
    r"C:\Program Files\RyzenAI\1.8",
    r"C:\Program Files\RyzenAI",
)


def _spec_path(spec: object) -> str | None:
    if spec is None:
        return None
    origin = getattr(spec, "origin", None)
    if isinstance(origin, str) and origin and origin != "namespace":
        return origin
    locations = getattr(spec, "submodule_search_locations", None)
    if locations:
        first = next(iter(locations), None)
        if isinstance(first, str) and first:
            return first
    return None


def _existing_dir(path: str | None) -> str | None:
    if not path or not os.path.isdir(path):
        return None
    return path


def _discover_path() -> str | None:
    for key in _INSTALL_ENV:
        found = _existing_dir(os.environ.get(key))
        if found is not None:
            return found
    for candidate in _INSTALL_DIRS:
        found = _existing_dir(candidate)
        if found is not None:
            return found
    return None


def _probe_module(name: str) -> dict:
    record = {"module": name, "imported": False, "path": None, "error": None}
    try:
        spec = importlib.util.find_spec(name)
    except (ImportError, ValueError, AttributeError) as exc:
        record["error"] = type(exc).__name__
        return record
    record["path"] = _spec_path(spec)
    if spec is None:
        return record
    try:
        module = importlib.import_module(name)
    except Exception as exc:
        record["error"] = type(exc).__name__
        return record
    origin = getattr(module, "__file__", None)
    if isinstance(origin, str) and origin:
        record["path"] = origin
    record["imported"] = True
    return record


def probe() -> dict:
    """Return a structured result. Does not raise. Does not download.

    generated is true only when this process actually produced tokens.
    The result contains no token text.
    """
    modules = []
    try:
        for name in _MODULES:
            modules.append(_probe_module(name))
    except Exception:
        modules.append({"module": "probe", "imported": False, "path": None, "error": "probe_failed"})
    imported = [item for item in modules if item.get("imported")]
    path = None
    for item in imported:
        if item.get("path"):
            path = item["path"]
            break
    if path is None:
        for item in modules:
            if item.get("path"):
                path = item["path"]
                break
    if path is None:
        path = _discover_path()
    # Weights are not loaded here, and no generator is invoked.
    weights_loaded = False
    generated = False
    token_count = 0
    if imported and not weights_loaded:
        message = "runtime import succeeded; model was not loaded; no tokens generated"
    else:
        message = "model was not loaded"
    return {
        "loaded": weights_loaded,
        "import_ok": bool(imported),
        "weights_loaded": weights_loaded,
        "generated": generated,
        "token_count": token_count,
        "path": path,
        "modules": modules,
        "download_attempted": False,
        "message": message,
    }
