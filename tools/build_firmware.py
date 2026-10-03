"""Build the firmware and stage it for OTA with the version read from the source.

The image name and the version compiled into it must always agree; doing that
by hand is what let an older binary be uploaded under a newer name.

    python tools/build_firmware.py

Steps: read FW_VERSION from src/config.h, run `pio run`, copy the result to
server/firmware/uwb-node-<version>.bin and verify the version is really inside
the image.
"""
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CONFIG = ROOT / "src" / "config.h"
BUILD = ROOT / ".pio" / "build" / "esp32uwb" / "firmware.bin"
DEST_DIR = ROOT / "server" / "firmware"


def source_version() -> str:
    m = re.search(r'#define\s+FW_VERSION\s+"([^"]+)"', CONFIG.read_text(encoding="utf-8"))
    if not m:
        sys.exit("FW_VERSION not found in src/config.h")
    return m.group(1)


def main() -> int:
    version = source_version()
    print(f"[build] source version: {version}")

    print("[build] running pio run ...")
    rc = subprocess.call([sys.executable, "-m", "platformio", "run"], cwd=ROOT)
    if rc != 0:
        sys.exit(f"build failed (exit {rc})")

    if not BUILD.is_file():
        sys.exit(f"missing {BUILD}")

    DEST_DIR.mkdir(parents=True, exist_ok=True)
    dest = DEST_DIR / f"uwb-node-{version}.bin"
    shutil.copy2(BUILD, dest)

    # verify the version really is inside the image
    blob = dest.read_bytes()
    if version.encode() not in blob:
        sys.exit(f"version {version} not found inside {dest.name} — refusing to stage it")

    print(f"[build] staged {dest.name} ({dest.stat().st_size} bytes) — version verified in image")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
