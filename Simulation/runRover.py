"""Run the rover through Isaac Lab's Newton backend using only the CPU."""

import argparse
from contextlib import ExitStack
import time
from types import SimpleNamespace

from _support import runtime as cpu_runtime
import torch

import isaaclab.sim as sim_utils
from isaaclab.app import launch_simulation
from isaaclab.utils import instantiate
from isaaclab_newton.physics import MJWarpSolverCfg, NewtonCfg

from manualController import RoverController
from buildRover.makeRover import ROVER_USD, build_rover
from scene import RoverSceneCfg
from _support.physics import CpuMJWarpSolver

DT = 1.0 / 120.0


def differential_drive(v, yaw_rate):
    # Wheel collision radius and left/right axle spacing from lunabot_2627.urdf.
    radius = 0.14796
    track_width = 0.541191
    return (v - yaw_rate * track_width / 2) / radius, (v + yaw_rate * track_width / 2) / radius


def run_simulator(sim, scene, max_steps, realtime=False):
    controller = RoverController()
    robot = scene["robot"]
    left_ids = [robot.joint_names.index(name) for name in ("wheel_front_left_joint", "wheel_rear_left_joint")]
    right_ids = [robot.joint_names.index(name) for name in ("wheel_front_right_joint", "wheel_rear_right_joint")]
    dt = sim.get_physics_dt()
    step = 0
    wall_start = time.monotonic()
    while sim.is_running() and (max_steps == 0 or step < max_steps):
        v, yaw_rate = controller.update(robot, scene["camera"], scene["lidar"], dt)
        left_speed, right_speed = differential_drive(v, yaw_rate)
        targets = torch.zeros((robot.num_instances, robot.num_joints), device="cpu")
        targets[:, left_ids] = left_speed
        targets[:, right_ids] = right_speed
        robot.actuators.target_command.set_velocity_index(value=targets)
        scene.write_data_to_sim()
        sim.step()
        scene.update(dt)
        step += 1
        if realtime:
            # Pace visualizers near real time instead of racing past the scene.
            delay = wall_start + step * dt - time.monotonic()
            if delay > 0:
                time.sleep(delay)
        if step % 120 == 0:
            print(f"time={step * dt:.1f}s position={robot.data.root_pos_w.torch.tolist()}", flush=True)

    cpu_runtime.verify_cpu_runtime()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=0, help="Stop after N steps; 0 runs until Ctrl+C.")
    parser.add_argument("--num-envs", type=int, choices=[1], default=1, help="Native CPU solver supports one world.")
    parser.add_argument("--device", choices=["cpu"], default="cpu")
    parser.add_argument("--viz", choices=["none", "newton_gl"], default="newton_gl")
    args = parser.parse_args()
    if args.steps < 0 or args.num_envs < 1:
        parser.error("--steps must be non-negative and --num-envs must be positive")

    if not ROVER_USD.is_file():
        build_rover()

    sim_cfg = sim_utils.SimulationCfg(
        dt=DT,
        device="cpu",
        use_fabric=False,
        physics=NewtonCfg(
            use_cuda_graph=False,
            solver_cfg=MJWarpSolverCfg(
                class_type=CpuMJWarpSolver,
                use_mujoco_cpu=True,
                # Integrate velocity-servo damping implicitly. Euler lets the
                # high-gain wheel drives overshoot and kick the chassis upward.
                integrator="implicitfast",
            ),
        ),
    )
    scene_cfg = RoverSceneCfg(num_envs=args.num_envs, env_spacing=10.0)
    if args.viz == "newton_gl":
        from _support.opengl import close_windows, prepare_software_opengl, verify_software_opengl
        from isaaclab.visualizers import PerspectiveCameraCfg, SceneCameraCfg, VisualizerCfg, WindowCfg
        from isaaclab_visualizers.newton import NewtonGLBackendCfg

        prepare_software_opengl()
        sim_cfg.visualizer_cfgs = [VisualizerCfg(
            backend=NewtonGLBackendCfg(enable_shadows=False, enable_picking=False),
            window=WindowCfg(size=(1100, 720), fps=30),
            cameras=[
                PerspectiveCameraCfg(eye=(1.8, -2.4, 1.5), lookat=(0.0, 0.0, 0.1)),
                SceneCameraCfg(prim_path=scene_cfg.camera.prim_path),
            ],
        )]
    # Include the scene in launch validation so the camera resolves to Newton, never Kit.
    with launch_simulation(
        SimpleNamespace(sim=sim_cfg, scene=scene_cfg),
        {"visualizer": "newton_gl" if args.viz == "newton_gl" else "none", "device": "cpu"},
    ), ExitStack() as cleanup:
        if args.viz == "newton_gl":
            cleanup.callback(close_windows)
        sim = sim_utils.SimulationContext(sim_cfg)
        scene = instantiate(scene_cfg)
        sim.reset()
        if args.viz == "newton_gl":
            verify_software_opengl()
        print("CPU-only Isaac Lab rover initialized.", flush=True)
        run_simulator(
            sim, scene, args.steps,
            realtime=args.viz != "none",
        )


if __name__ == "__main__":
    main()
