
from pathlib import Path

# Rover dimensions, in meters
BODY_LENGTH = 0.60
BODY_WIDTH = 0.40
BODY_HEIGHT = 0.15

WHEEL_RADIUS = 0.10
WHEEL_WIDTH = 0.06

# Wheel centers, relative to the chassis
WHEELS = {
    "front_left":  (0.20,  0.23),
    "front_right": (0.20, -0.23),
    "rear_left":  (-0.20,  0.23),
    "rear_right": (-0.20, -0.23),
}

urdf = f"""<?xml version="1.0"?>
<robot name="rover">

  <material name="blue">
    <color rgba="0.15 0.35 0.8 1"/>
  </material>

  <material name="black">
    <color rgba="0.1 0.1 0.1 1"/>
  </material>

  <link name="base_link">
    <visual>
      <geometry>
        <box size="{BODY_LENGTH} {BODY_WIDTH} {BODY_HEIGHT}"/>
      </geometry>
      <material name="blue"/>
    </visual>

    <collision>
      <geometry>
        <box size="{BODY_LENGTH} {BODY_WIDTH} {BODY_HEIGHT}"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="8.0"/>
      <inertia ixx="0.12" ixy="0" ixz="0"
               iyy="0.25" iyz="0" izz="0.35"/>
    </inertial>
  </link>
"""

for name, (x, y) in WHEELS.items():
    urdf += f"""
  <link name="{name}_wheel">
    <visual>
      <origin rpy="1.57079632679 0 0"/>
      <geometry>
        <cylinder radius="{WHEEL_RADIUS}" length="{WHEEL_WIDTH}"/>
      </geometry>
      <material name="black"/>
    </visual>

    <collision>
      <origin rpy="1.57079632679 0 0"/>
      <geometry>
        <cylinder radius="{WHEEL_RADIUS}" length="{WHEEL_WIDTH}"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.6"/>
      <inertia ixx="0.00168" ixy="0" ixz="0"
               iyy="0.003" iyz="0" izz="0.00168"/>
    </inertial>
  </link>

  <joint name="{name}_joint" type="continuous">
    <parent link="base_link"/>
    <child link="{name}_wheel"/>
    <origin xyz="{x} {y} 0"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="0.05" friction="0"/>
  </joint>
"""

urdf += "</robot>\n"

output = Path(__file__).parent / "rover.urdf"
output.write_text(urdf)

print(f"Created: {output}")
