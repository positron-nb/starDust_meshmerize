"""Build the robot core (firmware/src minus the ESP32 HAL) as a host shared
library for the simulator and the unit tests.

The SAME C++ files are compiled into the ESP32 firmware by PlatformIO; only
src/main.cpp and src/hal_esp32/ are excluded here, and src/sim_api/ is
included (it is empty unless MESH_SIM is defined).

Compiler selection (first that works):
  1. $CXX if set
  2. `python -m ziglang c++`  (pip install ziglang - portable, 64-bit on Windows)
  3. g++ / clang++ on PATH

Usage:  python tools/build_core.py [--debug] [--cxx "g++"]
"""
from __future__ import annotations

import argparse
import os
import platform
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "firmware" / "src"
INC = ROOT / "firmware" / "include"
OUT_DIR = ROOT / "simulator" / "robot" / "_build"
EXCLUDE_DIRS = {"hal_esp32"}
EXCLUDE_FILES = {"main.cpp"}


def lib_name() -> str:
    if sys.platform.startswith("win"):
        return "meshcore.dll"
    if sys.platform == "darwin":
        return "libmeshcore.dylib"
    return "libmeshcore.so"


def sources() -> list[Path]:
    out = []
    for p in sorted(SRC.rglob("*.cpp")):
        rel = p.relative_to(SRC)
        if rel.parts[0] in EXCLUDE_DIRS or rel.name in EXCLUDE_FILES:
            continue
        out.append(p)
    return out


def candidate_compilers(explicit: str | None) -> list[list[str]]:
    c: list[list[str]] = []
    if explicit:
        c.append(shlex.split(explicit))
    if os.environ.get("CXX"):
        c.append(shlex.split(os.environ["CXX"]))
    try:
        import ziglang  # noqa: F401

        c.append([sys.executable, "-m", "ziglang", "c++"])
    except ImportError:
        pass
    for name in ("g++", "clang++"):
        if shutil.which(name):
            c.append([name])
    return c


def build(debug: bool = False, cxx: str | None = None, quiet: bool = False) -> Path:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out = OUT_DIR / lib_name()
    srcs = sources()
    flags = [
        "-std=c++17",
        "-DMESH_SIM=1",
        f"-I{INC}",
        "-shared",
        "-fPIC",
        "-Wall",
        "-Wextra",
        "-Wno-unused-parameter",
        "-Wno-missing-field-initializers",
        "-O0" if debug else "-O2",
        "-g" if debug else "-s",
    ]
    if sys.platform.startswith("win"):
        flags.append("-target")
        flags.append("x86_64-windows-gnu" if platform.machine().endswith("64") else "x86-windows-gnu")
    errors = []
    for comp in candidate_compilers(cxx):
        cmd = comp + [str(s) for s in srcs] + flags + ["-o", str(out)]
        if "ziglang" not in " ".join(comp) and "-target" in cmd:
            i = cmd.index("-target")
            del cmd[i : i + 2]
        if not quiet:
            print("building core with:", " ".join(comp))
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode == 0:
            if r.stderr.strip() and not quiet:
                print(r.stderr)
            if not quiet:
                print("built", out)
            return out
        errors.append((" ".join(comp), r.stderr[-6000:]))
    msg = "\n\n".join(f"[{c}]\n{e}" for c, e in errors) or "no C++ compiler found (pip install ziglang)"
    raise RuntimeError("core build failed:\n" + msg)


def needs_rebuild() -> bool:
    out = OUT_DIR / lib_name()
    if not out.exists():
        return True
    t = out.stat().st_mtime
    files = list(SRC.rglob("*.cpp")) + list(SRC.rglob("*.h")) + list(INC.rglob("*.h"))
    return any(f.stat().st_mtime > t for f in files)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--debug", action="store_true")
    ap.add_argument("--cxx", default=None)
    a = ap.parse_args()
    try:
        build(a.debug, a.cxx)
    except RuntimeError as e:
        print(e)
        sys.exit(1)
