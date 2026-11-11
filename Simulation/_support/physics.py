"""Isaac Lab adapter for Newton's native MuJoCo CPU solver."""

from isaaclab_newton.physics.mjwarp_solver import MJWarpSolver


class CpuMJWarpSolver(MJWarpSolver):
    # Newton's CPU solver handles collision response internally, but does not
    # implement its optional contact-export API. This rover has no contact sensor.
    contact_sensors = False

    @classmethod
    def create_contacts(cls, cfg, solver):
        if not cfg.use_mujoco_cpu or not cfg.use_mujoco_contacts:
            raise ValueError("CPU rover requires native MuJoCo CPU contacts.")
        return None
