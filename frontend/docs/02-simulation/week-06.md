# Week 6: Robot Modeling with URDF & Gazebo Basics

## Overview

This week introduces robot simulation with URDF (Unified Robot Description Format) and Gazebo Classic. You'll learn how to describe robot geometry, physics, and sensors in URDF, visualize robots in RViz, and simulate them in Gazebo's physics engine.

## Learning Objectives

By the end of this week, you will be able to:

- Understand URDF format for robot description
- Create robot models with links, joints, and visual/collision geometry
- Add physics properties (mass, inertia, friction)
- Integrate sensors (cameras, lidar, IMU) into robot models
- Visualize robots in RViz
- Simulate robots in Gazebo Classic
- Control simulated robots via ROS 2 topics

## Why Robot Simulation?

Before deploying code on expensive hardware, simulation provides:

**Benefits:**
- **Safety**: Test dangerous scenarios (falling, collisions) without risk
- **Speed**: Iterate faster than real-time hardware testing
- **Cost**: No hardware wear-and-tear or breakage
- **Reproducibility**: Exact same conditions for debugging
- **Parallel testing**: Run multiple simulations simultaneously
- **Data generation**: Synthetic datasets for ML training

**Limitations:**
- **Sim-to-real gap**: Physics approximations differ from reality
- **Sensor modeling**: Cameras, lidar have idealized behavior
- **Contact dynamics**: Friction, deformation, grasping are simplified
- **Computational cost**: High-fidelity simulation requires GPU

## URDF: Unified Robot Description Format

URDF is an XML format for describing robot kinematics, dynamics, and visualization.

### URDF Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot">

  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>      <!-- How it looks -->
    <collision>   <!-- Collision geometry -->
    <inertial>    <!-- Mass and inertia -->
  </link>

  <!-- Joints connect links -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <axis xyz="0 0 1"/>
    <limit effort="10" velocity="1.0" lower="-1.57" upper="1.57"/>
  </joint>

</robot>
```

### Links: Robot Components

A **link** represents a rigid body (chassis, wheel, arm segment).

```xml
<link name="chassis">
  <!-- Visual: What you see in RViz/Gazebo -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.5 0.3 0.1"/>  <!-- Width, depth, height -->
    </geometry>
    <material name="blue">
      <color rgba="0 0 0.8 1"/>  <!-- RGBA -->
    </material>
  </visual>

  <!-- Collision: For physics simulation -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.5 0.3 0.1"/>  <!-- Often same as visual -->
    </geometry>
  </collision>

  <!-- Inertial: Mass properties -->
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="5.0"/>  <!-- kg -->
    <inertia ixx="0.02" ixy="0" ixz="0"
             iyy="0.05" iyz="0"
             izz="0.06"/>
  </inertial>
</link>
```

**Geometry primitives:**
- `<box size="x y z"/>` - Rectangular box
- `<cylinder radius="r" length="l"/>` - Cylinder
- `<sphere radius="r"/>` - Sphere
- `<mesh filename="model.dae"/>` - 3D mesh file

### Joints: Connecting Links

Joints define how links move relative to each other.

**Joint types:**

| Type | DOF | Description | Example |
|------|-----|-------------|---------|
| **fixed** | 0 | No movement | Camera mount |
| **revolute** | 1 | Rotation with limits | Robot arm joint |
| **continuous** | 1 | Unlimited rotation | Wheel |
| **prismatic** | 1 | Linear motion | Elevator, gripper |
| **planar** | 2 | 2D motion | Mobile base (x, y) |
| **floating** | 6 | Free motion | Drone |

**Example: Revolute joint (robot arm)**

```xml
<joint name="shoulder_joint" type="revolute">
  <parent link="base_link"/>
  <child link="upper_arm"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>  <!-- Rotate around Y-axis -->
  <limit effort="100" velocity="1.0" lower="-1.57" upper="1.57"/>
  <dynamics damping="0.7" friction="0.0"/>
</joint>
```

**Example: Continuous joint (wheel)**

```xml
<joint name="left_wheel_joint" type="continuous">
  <parent link="chassis"/>
  <child link="left_wheel"/>
  <origin xyz="-0.1 0.2 0" rpy="1.57 0 0"/>  <!-- Rotate 90° to align -->
  <axis xyz="0 0 1"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

### Computing Inertia

For basic shapes, use these formulas:

**Box (width w, depth d, height h, mass m):**
```
Ixx = (1/12) * m * (d² + h²)
Iyy = (1/12) * m * (w² + h²)
Izz = (1/12) * m * (w² + d²)
```

**Cylinder (radius r, length l, mass m):**
```
Ixx = Iyy = (1/12) * m * (3r² + l²)
Izz = (1/2) * m * r²
```

**Sphere (radius r, mass m):**
```
Ixx = Iyy = Izz = (2/5) * m * r²
```

**Python helper:**
```python
def box_inertia(m, w, d, h):
    """Compute inertia matrix for box."""
    return {
        'ixx': (1/12) * m * (d**2 + h**2),
        'iyy': (1/12) * m * (w**2 + h**2),
        'izz': (1/12) * m * (w**2 + d**2),
        'ixy': 0, 'ixz': 0, 'iyz': 0
    }

# Example: 5kg box (0.5m x 0.3m x 0.1m)
inertia = box_inertia(5.0, 0.5, 0.3, 0.1)
# ixx=0.02, iyy=0.05, izz=0.06
```

## Building a Differential Drive Robot

Let's build a complete 2-wheeled robot with caster.

### Robot Design

```
     ┌───────────┐
     │  Chassis  │  (box: 0.5m x 0.3m x 0.1m)
     └─┬──────┬──┘
       │      │
   ┌───┴──┐ ┌┴───┐
   │ Left │ │Right│  (wheels: radius 0.1m)
   │Wheel │ │Wheel│
   └──────┘ └─────┘
       │
    ┌──┴──┐
    │Caster│  (sphere: radius 0.05m)
    └─────┘
```

### Complete URDF

**`robot.urdf`:**

```xml
<?xml version="1.0"?>
<robot name="diff_drive_robot">

  <!-- Base Link (required, often just reference frame) -->
  <link name="base_link"/>

  <!-- Chassis -->
  <link name="chassis">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.06"/>
    </inertial>
  </link>

  <joint name="base_to_chassis" type="fixed">
    <parent link="base_link"/>
    <child link="chassis"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- Left Wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0025"/>
    </inertial>
  </link>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="chassis"/>
    <child link="left_wheel"/>
    <origin xyz="-0.1 0.175 0" rpy="-1.57 0 0"/>  <!-- Rotate to align cylinder -->
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Right Wheel (mirror of left) -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0025"/>
    </inertial>
  </link>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="chassis"/>
    <child link="right_wheel"/>
    <origin xyz="-0.1 -0.175 0" rpy="-1.57 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Caster Wheel (passive) -->
  <link name="caster">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="caster_joint" type="fixed">
    <parent link="chassis"/>
    <child link="caster"/>
    <origin xyz="0.2 0 -0.05" rpy="0 0 0"/>
  </joint>

</robot>
```

## Visualizing in RViz

RViz is ROS 2's 3D visualization tool.

### Step 1: Install Joint State Publisher

```bash
sudo apt install ros-humble-joint-state-publisher-gui -y
```

### Step 2: Create Launch File

**`urdf_visualize.launch.py`:**

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get URDF file path
    urdf_file = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        'robot.urdf'
    )

    # Read URDF file
    with open(urdf_file, 'r') as file:
        robot_desc = file.read()

    return LaunchDescription([
        # Robot State Publisher (publishes TF transforms)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),

        # Joint State Publisher GUI (control joints manually)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])
```

### Step 3: Launch and Configure RViz

```bash
ros2 launch my_robot_description urdf_visualize.launch.py
```

**In RViz:**
1. Set **Fixed Frame** to `base_link`
2. Click **Add** → **RobotModel**
3. Move joint sliders in Joint State Publisher GUI
4. Robot should appear and move!

## Gazebo Classic Integration

Gazebo simulates physics, sensors, and actuators.

### Adding Gazebo-Specific Tags

Gazebo requires additional XML tags in URDF for materials and physics:

```xml
<!-- Add to each link for Gazebo materials -->
<gazebo reference="chassis">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>  <!-- Friction coefficient -->
  <mu2>0.2</mu2>
</gazebo>

<gazebo reference="left_wheel">
  <material>Gazebo/Black</material>
  <mu1>1.0</mu1>  <!-- High friction for wheels -->
  <mu2>1.0</mu2>
</gazebo>
```

**Common Gazebo materials:**
- `Gazebo/Red`, `Gazebo/Blue`, `Gazebo/Green`
- `Gazebo/Black`, `Gazebo/White`, `Gazebo/Grey`
- `Gazebo/Orange`, `Gazebo/Yellow`

### Differential Drive Plugin

To control the robot, add a Gazebo plugin:

```xml
<!-- Add at end of URDF, inside <robot> -->
<gazebo>
  <plugin name="diff_drive_controller" filename="libgazebo_ros_diff_drive.so">
    <!-- Wheel joints -->
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>

    <!-- Wheel separation and diameter -->
    <wheel_separation>0.35</wheel_separation>
    <wheel_diameter>0.2</wheel_diameter>

    <!-- Command topic (subscribes to Twist) -->
    <command_topic>cmd_vel</command_topic>

    <!-- Odometry topic and frame -->
    <odometry_topic>odom</odometry_topic>
    <odometry_frame>odom</odometry_frame>
    <robot_base_frame>base_link</robot_base_frame>

    <!-- Publish odometry -->
    <publish_odom>true</publish_odom>
    <publish_odom_tf>true</publish_odom_tf>
    <publish_wheel_tf>false</publish_wheel_tf>

    <!-- Update rate -->
    <update_rate>50</update_rate>
  </plugin>
</gazebo>
```

### Launching Gazebo

**`gazebo_launch.py`:**

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Path to URDF
    urdf_file = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        'robot.urdf'
    )

    with open(urdf_file, 'r') as file:
        robot_desc = file.read()

    # Include Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch'),
            '/gazebo.launch.py'
        ])
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_robot',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.2'
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_pub,
        spawn_robot,
    ])
```

### Running Simulation

```bash
# Launch Gazebo with robot
ros2 launch my_robot_description gazebo_launch.py

# Control robot (separate terminal)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Or publish directly
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.2}}"
```

## Common URDF Issues

### Issue 1: Robot Falls Through Ground
**Cause**: No collision geometry or incorrect inertia
**Fix**: Add `<collision>` tags matching visual geometry

### Issue 2: Robot Explodes/Shakes Violently
**Cause**: Overlapping collision geometries or zero inertia
**Fix**: Ensure collision shapes don't overlap, add realistic mass/inertia

### Issue 3: Wheels Don't Rotate
**Cause**: Joint axis incorrect or plugin not loaded
**Fix**: Check joint axis direction (usually `xyz="0 0 1"` for wheels)

### Issue 4: Robot Doesn't Move in Gazebo
**Cause**: Plugin not loaded or topic mismatch
**Fix**: Verify plugin in URDF, check `ros2 topic list`

## Week 6 Hands-On Project

**Task**: Build a 4-wheeled rover with camera

**Requirements:**
1. URDF with chassis, 4 wheels (continuous joints), camera link
2. Proper mass and inertia for all links
3. Gazebo materials and friction coefficients
4. Differential drive plugin (treat as 2 wheels + 2 passive)
5. Camera sensor plugin (next section)
6. Launch file for RViz visualization
7. Launch file for Gazebo simulation
8. Demo video showing robot moving in Gazebo

## Resources

- [URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- [Gazebo Classic Documentation](https://classic.gazebosim.org/)
- [Gazebo ROS 2 Plugins](https://github.com/ros-simulation/gazebo_ros_pkgs)
- [URDF Validator](http://wiki.ros.org/urdf/Tutorials/Check%20URDF)
- [SolidWorks to URDF](http://wiki.ros.org/sw_urdf_exporter)

## Next Steps

Great job! You now know how to model robots in URDF and simulate them in Gazebo.

Next week: [Week 7: Sensors, Worlds, and Advanced Gazebo](week-07.md)

We'll add sensors (cameras, lidar), create custom worlds, and explore advanced simulation techniques!
