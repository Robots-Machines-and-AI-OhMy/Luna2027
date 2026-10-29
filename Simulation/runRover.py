import argparse
from pathlib import Path

from manualController import RoverController
from scene import RoverSceneCfg

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser()
add_launcher_args(parser)
args = parser.parse_args()
args.enable_cameras = True

import torch

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import (
    CameraCfg,
    MultiMeshRayCasterCfg,
    patterns,
)
from isaaclab.utils import configclass, instantiate

# IMPORTANT CONFIG:
controller = RoverController()
NUM_ENVS = 10
ENV_SPACING = 10
DT = 1.0 / 120.0 #Update 120 times per second



def differential_drive(v, yaw_rate):
    """Convert body velocity commands to wheel angular speeds."""
    radius = 0.10
    track_width = 0.46

    left = (v - yaw_rate * track_width / 2) / radius
    right = (v + yaw_rate * track_width / 2) / radius

    return left, right


def run_simulator(sim, scene):
    robot = scene["robot"]


    print("Wheel joint names:", robot.joint_names)

    # Find joints by name rather than relying on USD ordering.
    left_ids = [
        robot.joint_names.index("front_left_joint"),
        robot.joint_names.index("rear_left_joint"),
    ]
    right_ids = [
        robot.joint_names.index("front_right_joint"),
        robot.joint_names.index("rear_right_joint"),
    ]

    dt = sim.get_physics_dt()
    elapsed = 0.0
    step = 0

    while sim.is_running():

        # Simple demonstration:
        # four seconds forward, four seconds rotating.
        phase = int(elapsed // 4) % 2

        v, yaw_rate = controller.update(robot, scene["camera"], scene["lidar"], dt)

        left_speed, right_speed = differential_drive(
            v, yaw_rate
        )

        targets = torch.zeros(
            (robot.num_instances, robot.num_joints),
            device=robot.device,
        )

        targets[:, left_ids] = left_speed
        targets[:, right_ids] = right_speed

        # Isaac Lab 3 actuator command interface
        robot.actuators.target_command.set_velocity_index(
            value=targets
        )

        scene.write_data_to_sim()
        sim.step()
        scene.update(dt)

        elapsed += dt
        step += 1

        # Inspect sensor readings once per second
        if step % max(1, round(1.0 / dt)) == 0:
            rgb = scene["camera"].data.output["rgb"]

            hits = scene["lidar"].data.ray_hits_w.torch
            valid = torch.isfinite(hits).all(dim=-1)

            print(
                f"time={elapsed:.1f}s",
                f"camera={tuple(rgb.shape)}",
                f"lidar_hits={valid.sum().item()}",
            )


def main():
    sim_cfg = sim_utils.SimulationCfg(
        dt=DT,
        device=args.device,
    )

    with launch_simulation(sim_cfg, args):
        sim = sim_utils.SimulationContext(sim_cfg)

        sim.set_camera_view(
            eye=[3.5, 4.0, 3.0],
            target=[0.0, 0.0, 0.0],
        )

        scene = instantiate(
            RoverSceneCfg(
                num_envs=NUM_ENVS,
                env_spacing=ENV_SPACING,
            )
        )

        sim.reset()
        print("Simulation initialized.")

        run_simulator(sim, scene)


if __name__ == "__main__":
    main()
