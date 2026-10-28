"""ctypes bridge to the robot core compiled from firmware/src (meshcore.dll /
libmeshcore.so). Builds the library on first use if it is missing or stale.

The struct layouts below MUST match firmware/include/robot_types.h.
"""
from __future__ import annotations

import ctypes as C
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

MAX_SENSORS = 16

STATE_NAMES = [
    "BOOT", "IDLE", "CALIBRATING", "ARMED", "DRY_RUN", "GOAL_CONFIRMATION", "MAP_VALIDATION",
    "ROUTE_PLANNING", "WAIT_FOR_ACTUAL_RUN", "ACTUAL_RUN", "FINISHED", "RECOVERY", "FAULT", "CLEAR_MEMORY",
]
STAGE_NAMES = ["FRESH", "DRY_PARTIAL", "DRY_DONE", "ACTUAL_DONE"]
FAULT_NAMES = ["NONE", "LINE_LOST", "STALL", "LOW_BATTERY", "ROUTE_MISMATCH", "MAP_FULL", "NO_FRONTIER",
               "TURN_FAILED", "CALIBRATION_BAD", "STORAGE", "TIMEOUT"]


class HalInputs(C.Structure):
    _fields_ = [
        ("t_us", C.c_uint32),
        ("raw", C.c_uint16 * MAX_SENSORS),
        ("enc_left", C.c_int32),
        ("enc_right", C.c_int32),
        ("vbat", C.c_float),
        ("button", C.c_uint8),
        ("gyro_ok", C.c_uint8),
        ("pad", C.c_uint8 * 2),
        ("gyro_z_dps", C.c_float),
    ]


class HalOutputs(C.Structure):
    _fields_ = [
        ("duty_left", C.c_float),
        ("duty_right", C.c_float),
        ("led_red", C.c_uint8),
        ("led_status", C.c_uint8),
        ("emitters_on", C.c_uint8),
        ("motors_enable", C.c_uint8),
        ("save_request", C.c_uint8),
        ("pad", C.c_uint8 * 3),
    ]


_lib = None


def load_lib():
    global _lib
    if _lib is not None:
        return _lib
    import os

    import build_core  # tools/build_core.py

    frozen = os.environ.get("MESH_CORE_LIB")  # use a fixed library (no rebuild), e.g. for long studies
    if frozen:
        path = Path(frozen)
    else:
        if build_core.needs_rebuild():
            build_core.build(quiet=True)
        path = build_core.OUT_DIR / build_core.lib_name()
    lib = C.CDLL(str(path))
    vp = C.c_void_p
    sig = {
        "mesh_new": ([], vp),
        "mesh_free": ([vp], None),
        "mesh_param_count": ([], C.c_int),
        "mesh_param_name": ([C.c_int], C.c_char_p),
        "mesh_set_param": ([vp, C.c_char_p, C.c_double], C.c_int),
        "mesh_get_param": ([vp, C.c_char_p], C.c_double),
        "mesh_boot": ([vp], None),
        "mesh_step": ([vp, C.POINTER(HalInputs), C.POINTER(HalOutputs)], None),
        "mesh_state": ([vp], C.c_int),
        "mesh_stage": ([vp], C.c_int),
        "mesh_fault": ([vp], C.c_int),
        "mesh_run_kind": ([vp], C.c_int),
        "mesh_saves": ([vp], C.c_int),
        "mesh_storage_get": ([vp, C.c_char_p, C.c_int], C.c_int),
        "mesh_storage_set": ([vp, C.c_char_p, C.c_int], C.c_int),
        "mesh_pop_log": ([C.c_char_p, C.c_int], C.c_int),
        "mesh_clear_log": ([], None),
        "mesh_export_json": ([vp, C.c_char_p, C.c_int], C.c_int),
        "mesh_graph_new": ([], vp),
        "mesh_graph_free": ([vp], None),
        "mesh_graph_add_node": ([vp, C.c_float, C.c_float, C.c_int, C.c_int], C.c_int),
        "mesh_graph_set_goal": ([vp, C.c_int], None),
        "mesh_graph_add_edge": ([vp, C.c_int, C.c_int, C.c_int, C.c_int, C.c_float, C.c_int], C.c_int),
        "mesh_graph_match": ([vp, C.c_float, C.c_float, C.c_int, C.c_int, C.c_float], C.c_int),
        "mesh_graph_validate": ([vp], C.c_int),
        "mesh_plan": ([vp, C.c_int, C.c_int, C.c_int, C.c_float, C.POINTER(C.c_int), C.c_int,
                       C.POINTER(C.c_float), C.POINTER(C.c_int)], C.c_int),
        "mesh_lower_bound": ([vp, C.c_int, C.c_int, C.c_int], C.c_float),
        "mesh_nearest_frontier": ([vp, C.c_int, C.c_int], C.c_int),
        "mesh_explore_decide": ([vp, C.c_int, C.c_int, C.c_int, C.c_int], C.c_int),
        "mesh_persist_roundtrip": ([vp, C.c_int], C.c_int),
    }
    for name, (args, res) in sig.items():
        f = getattr(lib, name)
        f.argtypes = args
        f.restype = res
    _lib = lib
    return lib


class RobotCore:
    """One robot brain (firmware core) instance."""

    def __init__(self, params: dict | None = None):
        self.lib = load_lib()
        self.h = self.lib.mesh_new()
        self.inp = HalInputs()
        self.out = HalOutputs()
        self._jbuf = C.create_string_buffer(1 << 18)
        self._lbuf = C.create_string_buffer(256)
        if params:
            for k, v in params.items():
                self.set_param(k, v)

    def close(self):
        if self.h:
            self.lib.mesh_free(self.h)
            self.h = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

    def set_param(self, key: str, value: float) -> None:
        if self.lib.mesh_set_param(self.h, key.encode(), float(value)) != 0:
            raise KeyError(f"unknown firmware parameter {key}")

    def get_param(self, key: str) -> float:
        return self.lib.mesh_get_param(self.h, key.encode())

    def param_names(self) -> list[str]:
        return [self.lib.mesh_param_name(i).decode() for i in range(self.lib.mesh_param_count())]

    def boot(self) -> None:
        self.lib.mesh_boot(self.h)

    def step(self, t_us: int, raw, enc_l: int, enc_r: int, vbat: float, button: bool,
             gyro_dps: float = 0.0, gyro_ok: bool = False) -> HalOutputs:
        i = self.inp
        i.gyro_z_dps = gyro_dps
        i.gyro_ok = 1 if gyro_ok else 0
        i.t_us = t_us & 0xFFFFFFFF
        for k, v in enumerate(raw):
            i.raw[k] = int(v)
        i.enc_left = int(enc_l)
        i.enc_right = int(enc_r)
        i.vbat = vbat
        i.button = 1 if button else 0
        self.lib.mesh_step(self.h, C.byref(i), C.byref(self.out))
        return self.out

    @property
    def state(self) -> str:
        return STATE_NAMES[self.lib.mesh_state(self.h)]

    @property
    def stage(self) -> str:
        return STAGE_NAMES[self.lib.mesh_stage(self.h)]

    @property
    def fault(self) -> str:
        return FAULT_NAMES[self.lib.mesh_fault(self.h)]

    def storage(self) -> bytes:
        buf = C.create_string_buffer(64 * 1024)
        n = self.lib.mesh_storage_get(self.h, buf, len(buf))
        return buf.raw[:n]

    def set_storage(self, data: bytes) -> None:
        self.lib.mesh_storage_set(self.h, data, len(data))

    def export(self) -> dict:
        n = self.lib.mesh_export_json(self.h, self._jbuf, len(self._jbuf))
        if n < 0:
            raise RuntimeError("export buffer too small")
        return json.loads(self._jbuf.value[:n].decode())

    def pop_logs(self) -> list[str]:
        out = []
        while self.lib.mesh_pop_log(self._lbuf, len(self._lbuf)) > 0:
            out.append(self._lbuf.value.decode(errors="replace"))
        return out


class CoreGraph:
    """Direct access to the C++ MazeGraph + planner for unit tests."""

    def __init__(self):
        self.lib = load_lib()
        self.g = self.lib.mesh_graph_new()

    def __del__(self):
        try:
            self.lib.mesh_graph_free(self.g)
        except Exception:
            pass

    def add_node(self, x, y, exits: str, type_: int = 0) -> int:
        m = 0
        for ch in exits:
            m |= 1 << "NESW".index(ch)
        return self.lib.mesh_graph_add_node(self.g, x, y, m, type_)

    def set_goal(self, n):
        self.lib.mesh_graph_set_goal(self.g, n)

    def add_edge(self, a, da: str, b, db: str, length, corners=0) -> int:
        return self.lib.mesh_graph_add_edge(self.g, a, "NESW".index(da), b, "NESW".index(db), length, corners)

    def match(self, x, y, exits: str, arrive: str, radius: float) -> int:
        m = sum(1 << "NESW".index(ch) for ch in exits)
        return self.lib.mesh_graph_match(self.g, x, y, m, "NESW".index(arrive), radius)

    def validate(self) -> int:
        return self.lib.mesh_graph_validate(self.g)

    def plan(self, start, heading: str, goal, turn_weight=30.0):
        buf = (C.c_int * 300)()
        L = C.c_float()
        T = C.c_int()
        n = self.lib.mesh_plan(self.g, start, "NESW".index(heading), goal, turn_weight, buf, 100, C.byref(L), C.byref(T))
        if n < 0:
            return None
        steps = [(buf[3 * i], "NESW"[buf[3 * i + 1]], buf[3 * i + 2]) for i in range(n)]
        return {"steps": steps, "nodes": [s[0] for s in steps] + [goal], "length": L.value, "turns": T.value}

    def lower_bound(self, start, heading: str, goal) -> float:
        return self.lib.mesh_lower_bound(self.g, start, "NESW".index(heading), goal)

    def nearest_frontier(self, start, heading: str) -> int:
        return self.lib.mesh_nearest_frontier(self.g, start, "NESW".index(heading))

    def explore_decide(self, node, heading: str, policy=0, pref=0) -> str | None:
        d = self.lib.mesh_explore_decide(self.g, node, "NESW".index(heading), policy, pref)
        return "NESW"[d] if 0 <= d < 4 else None

    def persist_roundtrip(self, corrupt_offset=-1) -> int:
        return self.lib.mesh_persist_roundtrip(self.g, corrupt_offset)
