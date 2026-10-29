class RoverController:
    def __init__(self):
        pass

    def update(self, robot, camera, lidar, dt):
        # This is what is called every simulation step

        position = robot.data.root_pos_w
        rgb = camera.data.output["rgb"]
        lidar = lidar.data.ray_hits_w.torch

        velocity = 0.5
        yawRate = 0.2 #rads/s

        return velocity, yawRate