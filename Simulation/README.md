### Make the rover
The rover needs to be a USD that isaac lab can load. You can do this with the following:
1) Edit makeRover.py with the specs wanted. This is in urdf form, so google/AI it to find out how to make what you want with it.
2) Run makeRover.py. This will make `rover.urdf`.
3) From the IsaacLab directory (terminal should say ~.../IsaacLab), run `uv run isaaclab -p scripts/tools/convert_urdf.py <path_to_urdf> <path_to_save> --joint_stiffness 0.0 --joint_damping 25.0 --joint_target_type velocity`
4) In the generated folder, something called `rover.usda` will exist. 
5) Put the absolute path to that file in `scene.py` at the top.

---

### Editing the Scene
`scene.py` controls the scene.

Right now, I just have two obstacles being generated at constant positions. I used to have the actual lunabotics field,
but that was causing weird crashes.

#### TODO:
1) Modify scene so that it resembles the lunabotics competition field.
2) Modify scene so that rocks are randomly generated where they are allowed to be (make obstacles with randomly generated positions).
3) Modify the scene so it has uneven ground terrain. I don't think IsaacLab can do full "regolith" simulation, but it can do uneven ground terrains with low coefficients of friction.
3) Change `camera` and `lidar` to what is actually on rover (I'm assuming Jason/Johnny y'all have a better idea what's actually on the robot).

#### Notes:
Keep in mind that in order for an object to be registered by the LIDAR, it needs to be in the `lidar` `mesh_prim_paths`. 

---

### Controlling the Robot
`manualController.py` is an example.

#### How it Works

Every time the physics steps, it will call the <controller>.update function with the robot, camera, lidar, and dt (time step size).
I can also make it give the update the entire scene if we need to, we'll have to figure that out.

Right now, the `manualController.py` literally just has it mode at 0.5m/s at 0.2rad/s rotation, just so you can see how it works.
I also have it ingesting the position, rgb, and lidar data just so you can see how it does that.

As always, good old google can help with finding other capabilities of this.

#### TODO:
1) Start on the RL framework. The controller is where the main RL training will occur - training a controller.
2) Make the controller take in all data needed, for now I just have position, camera, lidar, and dt. It may need something else like goal position or something depending on how we do it.

---

### Running the Simulation
Now it's the fun part!

`runRover.py` is the main program. The important configs are at `#IMPORTANT CONFIG:`.

#### How it works
* Differential Drive (Skid Steer) takes in velocity and angle, outputs the left and right wheel velocity setpoints and applies it.
* Literally just steps continuously, updating commands from the controller.

#### How to Use:
From the Isaac Lab Directory, run the following command: \
`uv run isaaclab -p <path_to_runRover.py> --viz kit` 

That should launch the simulation and everything should work fine!

#### TODO:
1) Make the camera/lidar fit the actual robot. 
2) Make differential drive work as expected as far as mass and everything.
3) Not much to be honest... I'm sure we'll come up with more.
