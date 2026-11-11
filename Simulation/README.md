# Rover simulation

Isaac Lab with CPU physics and a native Newton viewer. No NVIDIA GPU or
Isaac Sim is needed. Supported setup: Windows x64 with
[Git](https://git-scm.com/downloads) and [uv](https://docs.astral.sh/uv/).

## Setup

From this folder, run once:

```powershell
uv run setup.py
```

`uv run` syncs the pinned Python dependencies automatically. The setup script
also installs software OpenGL inside `.venv`; plain `uv sync` installs only
the Python dependencies. Initial setup needs internet access.

## Run with the visualizer

```powershell
uv run python runRover.py --viz newton_gl
```

This opens the native Newton 3D window using CPU-only graphics. The visualizer
is the default, so `uv run python runRover.py` does the same thing.

Middle-drag to orbit, Shift + middle-drag to pan, and scroll to zoom.
The camera selector switches to the rover's front camera. Close the window
or press Ctrl+C to stop. The first run may take longer while kernels compile.

## Run headless

```powershell
# Run continuously without a window; stop with Ctrl+C
uv run python runRover.py --viz none

# Stop automatically after 240 physics steps (2 simulated seconds)
uv run python runRover.py --viz none --steps 240
```

Headless mode keeps the controller, camera and lidar active on the CPU, with
no display required. It runs as fast as the CPU allows rather than pacing to
real time. Use `--steps N` with either mode to limit the run.

## Edit the simulation

Edit `manualController.py` for driving logic, `scene.py` for the robot and
sensors, and `runRover.py` for the simulation loop. The controller returns
forward speed (m/s) and yaw rate (rad/s). Camera and lidar tensors are available
through `camera.data.output["rgb"].torch` and `lidar.data.ray_hits_w.torch`.

The ready-to-use robot is in `assets/rover/`. `buildRover/` contains the
optional URDF generator; changing its URDF does not regenerate the USD.
This configuration runs one rover per process, entirely on the CPU.
