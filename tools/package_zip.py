"""Package the repository as dist/meshmerize-robot-v<version>.zip, excluding
build outputs, caches, git metadata and previous archives.

  python tools/package_zip.py
"""
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VERSION = "1.1.0"
EXCLUDE_DIRS = {".git", ".pio", "_build", "_fig", "__pycache__", ".pytest_cache", "dist", ".vscode", "libdeps"}
EXCLUDE_SUFFIX = {".pyc", ".zip"}


def main():
    out = ROOT / "dist" / f"meshmerize-robot-v{VERSION}.zip"
    out.parent.mkdir(exist_ok=True)
    n = 0
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
        for p in sorted(ROOT.rglob("*")):
            rel = p.relative_to(ROOT)
            if any(part in EXCLUDE_DIRS for part in rel.parts) or p.suffix in EXCLUDE_SUFFIX or p.is_dir():
                continue
            z.write(p, Path("meshmerize-robot") / rel)
            n += 1
    with zipfile.ZipFile(out) as z:
        bad = z.testzip()
    print(f"wrote {out} ({n} files, {out.stat().st_size / 1e6:.2f} MB, integrity {'OK' if bad is None else 'BAD: ' + bad})")


if __name__ == "__main__":
    main()
