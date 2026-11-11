"""ctypes binding to the core shared library (C API in core/include/catena/catena_c.h)."""

from __future__ import annotations

import ctypes
import os
import sys
from functools import cache
from pathlib import Path
from typing import Any

from sim.config import CORE_FIELDS, REPO_ROOT, core_constants, core_params, load_config

IR_COUNT = core_constants(load_config())["CATENA_IR_COUNT"]
# Must match catena_c.h; load() checks the struct sizes against the library.
FRAME_MAX = 32
RX_MAX = 16
TX_MAX = 4
BROADCAST = 0xFF
FRAME_FLASH = 1

STATE_NAMES = {1: "WANDER"}
WANDER_DETAIL = {0: "cruise", 1: "avoid", 2: "backoff", 3: "turn"}

u8, u32, i32, f32 = ctypes.c_uint8, ctypes.c_uint32, ctypes.c_int32, ctypes.c_float


class Params(ctypes.Structure):
    """Mirror of catena_params_t. Built from the same field table as the generated header."""

    _fields_ = [(f.name, f32) for f in CORE_FIELDS]


class Rx(ctypes.Structure):
    _fields_ = [
        ("src", u8),
        ("len", u8),
        ("has_ir", u8),
        ("reserved", u8),
        ("ir", f32 * IR_COUNT),
        ("data", u8 * FRAME_MAX),
    ]


class Inputs(ctypes.Structure):
    _fields_ = [
        ("dt_us", u32),
        ("entropy", u32),
        ("gyro_z_rad_s", f32),
        ("ring_angle_rad", f32),
        ("hall", f32),
        ("ticks_left", i32),
        ("ticks_right", i32),
        ("table_left", u8),
        ("table_right", u8),
        ("rx_count", u8),
        ("reserved", u8),
        ("rx", Rx * RX_MAX),
    ]


class Tx(ctypes.Structure):
    _fields_ = [("dst", u8), ("len", u8), ("reserved", u8 * 2), ("data", u8 * FRAME_MAX)]


class Debug(ctypes.Structure):
    _fields_ = [
        ("state", u8),
        ("detail", u8),
        ("index", u8),
        ("chain_size", u8),
        ("neighbours", u8),
        ("reserved", u8 * 3),
    ]


class Outputs(ctypes.Structure):
    _fields_ = [
        ("wheel_left_mm_s", f32),
        ("wheel_right_mm_s", f32),
        ("flash", u8),
        ("flash_len", u8),
        ("tx_count", u8),
        ("reserved", u8),
        ("flash_delay_us", u32),
        ("flash_data", u8 * FRAME_MAX),
        ("tx", Tx * TX_MAX),
        ("debug", Debug),
    ]


def library_path() -> Path:
    """Path of the shared library: $CATENA_LIB, else the `make core` output."""
    env = os.environ.get("CATENA_LIB")
    if env:
        return Path(env)
    name = "libcatena.dylib" if sys.platform == "darwin" else "libcatena.so"
    return REPO_ROOT / "build" / "core" / name


@cache
def load() -> ctypes.CDLL:
    path = library_path()
    if not path.exists():
        raise FileNotFoundError(f"{path} not found: run `make core` first")
    lib = ctypes.CDLL(str(path))
    lib.catena_default_params.argtypes = [ctypes.POINTER(Params)]
    lib.catena_default_params.restype = None
    lib.catena_create.argtypes = [u8, ctypes.POINTER(Params)]
    lib.catena_create.restype = ctypes.c_void_p
    lib.catena_step.argtypes = [ctypes.c_void_p, ctypes.POINTER(Inputs), ctypes.POINTER(Outputs)]
    lib.catena_step.restype = None
    lib.catena_destroy.argtypes = [ctypes.c_void_p]
    lib.catena_destroy.restype = None
    lib.catena_flash_estimate.argtypes = [
        ctypes.POINTER(Params),
        ctypes.POINTER(f32),
        ctypes.POINTER(f32),
        ctypes.POINTER(f32),
    ]
    lib.catena_flash_estimate.restype = ctypes.c_int
    for name, struct in (("params", Params), ("inputs", Inputs), ("outputs", Outputs)):
        fn = getattr(lib, f"catena_sizeof_{name}")
        fn.argtypes = []
        fn.restype = ctypes.c_size_t
        if fn() != ctypes.sizeof(struct):
            raise RuntimeError(f"{path}: {name} layout differs from sim/corelib.py")
    return lib


def default_params(lib: ctypes.CDLL) -> Params:
    """The parameters compiled into the library from config/robot.yaml."""
    params = Params()
    lib.catena_default_params(ctypes.byref(params))
    return params


def params_from_config(cfg: dict[str, Any]) -> Params:
    """Parameters from a loaded (possibly overridden) config, to pass to catena_create."""
    return Params(**core_params(cfg))


def flash_estimate(params: Params, ir: Any) -> tuple[float, float] | None:
    """Core's flash ranging on one IR sample: (bearing_rad, range_mm), or None."""
    samples = (f32 * IR_COUNT)(*ir)
    bearing, rng = f32(), f32()
    ok = load().catena_flash_estimate(
        ctypes.byref(params), samples, ctypes.byref(bearing), ctypes.byref(rng)
    )
    return (bearing.value, rng.value) if ok else None


class Core:
    """One robot's core instance."""

    def __init__(self, robot_id: int, params: Params) -> None:
        self._lib = load()
        self._params = params  # keep alive: core copies it, but be safe
        handle = self._lib.catena_create(robot_id, ctypes.byref(params))
        if not handle:
            raise MemoryError("catena_create failed")
        self._handle = handle
        self.inputs = Inputs()
        self.outputs = Outputs()

    def step(self) -> Outputs:
        self._lib.catena_step(self._handle, ctypes.byref(self.inputs), ctypes.byref(self.outputs))
        return self.outputs

    def __del__(self) -> None:
        handle = getattr(self, "_handle", None)
        if handle:
            self._lib.catena_destroy(handle)
            self._handle = None
