from _support import runtime as cpu_runtime  # Disable CUDA before any simulator imports.
from buildRover.makeRover import ROVER_USD
import isaaclab.sim as sim_utils
from isaaclab_newton.renderers import NewtonWarpRendererCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import (
    CameraCfg,
    MultiMeshRayCasterCfg,
    patterns,
)
from isaaclab.utils import configclass


# Scene Config
@configclass
class RoverSceneCfg(InteractiveSceneCfg):

    # Ground
    ground = AssetBaseCfg(
        prim_path="/World/Ground",
        spawn=sim_utils.CuboidCfg(
            size=(200.0, 200.0, 0.1),
            collision_props=sim_utils.CollisionPropertiesCfg(),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -0.05)),
    )

    # Lighting
    light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(
            intensity=3000.0,
        ),
    )

    # Our four-wheeled rover
    robot = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(ROVER_USD),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.17),
        ),
        actuators={
            "wheels": ImplicitActuatorCfg(
                joint_names_expr=["wheel_.*_joint"],
                stiffness=0.0,
                damping=25.0,
                joint_effort_limit=100.0,
                joint_velocity_limit=5.03,
            )
        },
    )

    # Two stationary obstacles
    obstacle_1 = AssetBaseCfg(
        prim_path="/World/Obstacle1",
        spawn=sim_utils.CuboidCfg(
            size=(0.6, 0.6, 0.7),
            collision_props=sim_utils.CollisionPropertiesCfg(),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(20.0, 0.4, 0.35),
        ),
    )

    obstacle_2 = AssetBaseCfg(
        prim_path="/World/Obstacle2",
        spawn=sim_utils.CuboidCfg(
            size=(0.5, 0.5, 0.6),
            collision_props=sim_utils.CollisionPropertiesCfg(),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(20.5, -1.0, 0.30),
        ),
    )

    # Front-facing RGB camera
    camera = CameraCfg(
        renderer_cfg=NewtonWarpRendererCfg(use_cuda_graph=False),
        prim_path="{ENV_REGEX_NS}/Robot/Geometry/base_link/front_camera",
        update_period=0.1,
        height=120,
        width=160,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0,
            horizontal_aperture=20.955,
            clipping_range=(0.1, 100.0),
        ),
        offset=CameraCfg.OffsetCfg(
            pos=(0.30, 0.0, 0.12),
            rot=(0.5, -0.5, 0.5, -0.5),
            convention="ros",
        ),
    )

    # 360-degree planar LiDAR
    lidar = MultiMeshRayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/Geometry/base_link",
        update_period=0.1,
        offset=MultiMeshRayCasterCfg.OffsetCfg(
            pos=(0.0, 0.0, 0.24),
        ),
        ray_alignment="yaw",
        pattern_cfg=patterns.LidarPatternCfg(
            channels=1,
            vertical_fov_range=(0.0, 0.0),
            horizontal_fov_range=(-180.0, 180.0),
            horizontal_res=1.0,
        ),
        mesh_prim_paths=[
            "/World/Obstacle1",
            "/World/Obstacle2",
        ],
        debug_vis=False,
    )
