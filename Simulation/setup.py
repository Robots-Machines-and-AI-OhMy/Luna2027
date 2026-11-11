"""Run once with `uv run setup.py` to install the CPU-only viewer."""

import hashlib
from importlib.util import find_spec
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from urllib.request import urlopen


def main():
    if sys.platform != "win32" or sys.maxsize <= 2**32:
        raise SystemExit("The CPU viewer setup currently supports Windows x64.")
    # Source-built Isaac Lab wheels omit this runtime configuration.
    isaaclab = Path(find_spec("isaaclab").origin).parent
    experience = isaaclab / "apps" / "isaaclab.python.kit"
    if not experience.is_file():
        url = "https://raw.githubusercontent.com/isaac-sim/IsaacLab/90a16f06685dbf7948262eafa628c470b4cdc78c/apps/isaaclab.python.kit"
        with urlopen(url, timeout=60) as response:
            data = response.read()
        experience.parent.mkdir(exist_ok=True)
        experience.write_bytes(data)
    mesa = Path(sys.prefix) / "mesa"
    if not (mesa / "x64" / "opengl32.dll").is_file():
        if not shutil.which("tar"):
            raise SystemExit("Windows tar.exe is required to install the CPU viewer.")
        print("Installing CPU-only OpenGL...", flush=True)
        url = "https://github.com/pal1000/mesa-dist-win/releases/download/26.2.2/mesa3d-26.2.2-release-msvc.7z"
        checksum = "0f21394d44036019bb5c07a3788dd278fd7b7b3469077a4686f0d639e8e874a1"
        with tempfile.TemporaryDirectory() as temporary:
            archive = Path(temporary) / "mesa.7z"
            with urlopen(url, timeout=120) as response, archive.open("wb") as target:
                shutil.copyfileobj(response, target)
            with archive.open("rb") as source:
                actual_checksum = hashlib.file_digest(source, "sha256").hexdigest()
            if actual_checksum != checksum:
                raise SystemExit("Mesa download checksum mismatch; rerun setup.")
            mesa.mkdir(exist_ok=True)
            subprocess.run(["tar", "-xf", str(archive), "-C", str(mesa)], check=True)
        if not (mesa / "x64" / "opengl32.dll").is_file():
            raise SystemExit("Software OpenGL installation failed.")
    print("Ready. Run: uv run python runRover.py")


if __name__ == "__main__":
    main()
