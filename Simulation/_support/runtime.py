"""Disable CUDA before importing Isaac Lab or initializing Warp."""

import os
import subprocess
import sys

if os.environ.get("CUDA_VISIBLE_DEVICES") != "-1":
    # Windows native libraries can retain the process-start environment.
    # Restart before loading Warp so CUDA sees the restriction from the outset.
    environment = dict(os.environ, CUDA_VISIBLE_DEVICES="-1", NVIDIA_VISIBLE_DEVICES="void", LIVESTREAM="0")
    raise SystemExit(subprocess.call([sys.executable, *sys.argv], env=environment))

os.environ["CUDA_VISIBLE_DEVICES"] = "-1"
os.environ["NVIDIA_VISIBLE_DEVICES"] = "void"
os.environ["LIVESTREAM"] = "0"

import warp as wp

wp.init()
wp.set_device("cpu")

import torch

if torch.version.cuda is not None:
    raise RuntimeError("Use the CPU environment: uv run python runRover.py")


def verify_cpu_runtime():
    """Fail if any CUDA device became available during the simulation."""
    if any(device.is_cuda for device in wp.get_devices()):
        raise RuntimeError("CUDA is enabled; this runner requires CPU-only execution.")
    if torch.version.cuda is not None:
        raise RuntimeError("A CUDA-enabled PyTorch build was loaded.")


verify_cpu_runtime()
