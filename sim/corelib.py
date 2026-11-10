"""ctypes binding to the core shared library (C API in core/include/catena/catena_c.h)."""

from __future__ import annotations

import ctypes
import os
import sys
from pathlib import Path
from typing import Any

from sim.config import CORE_FIELDS, REPO_ROOT, core_params


class Params(ctypes.Structure):
    """Mirror of catena_params_t. Built from the same field table as the generated header."""

    _fields_ = [(f.name, ctypes.c_float) for f in CORE_FIELDS]


def library_path() -> Path:
    """Path of the shared library: $CATENA_LIB, else the `make core` output."""
    env = os.environ.get("CATENA_LIB")
    if env:
        return Path(env)
    name = "libcatena.dylib" if sys.platform == "darwin" else "libcatena.so"
    return REPO_ROOT / "build" / "core" / name


def load() -> ctypes.CDLL:
    path = library_path()
    if not path.exists():
        raise FileNotFoundError(f"{path} not found: run `make core` first")
    lib = ctypes.CDLL(str(path))
    lib.catena_default_params.argtypes = [ctypes.POINTER(Params)]
    lib.catena_default_params.restype = None
    lib.catena_sizeof_params.argtypes = []
    lib.catena_sizeof_params.restype = ctypes.c_size_t
    if lib.catena_sizeof_params() != ctypes.sizeof(Params):
        raise RuntimeError(f"{path} does not match sim/config.py: run `make core`")
    return lib


def default_params(lib: ctypes.CDLL) -> Params:
    """The parameters compiled into the library from config/robot.yaml."""
    params = Params()
    lib.catena_default_params(ctypes.byref(params))
    return params


def params_from_config(cfg: dict[str, Any]) -> Params:
    """Parameters from a loaded (possibly overridden) config, to pass to catena_create."""
    return Params(**core_params(cfg))
