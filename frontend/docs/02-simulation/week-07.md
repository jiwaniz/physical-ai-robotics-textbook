# Week 7: Sensors, Worlds, and Advanced Gazebo

## Overview

This week extends your Gazebo skills with sensor integration, custom world creation, and advanced simulation techniques. You'll add cameras, lidar, and IMUs to your robot, build realistic environments, and complete the Chapter 2 assessment project.

## Learning Objectives

By the end of this week, you will be able to:

- Integrate sensors (camera, lidar, IMU, GPS) into robot URDF
- Process sensor data via ROS 2 topics
- Create custom Gazebo worlds with models and terrain
- Use Gazebo model database and import custom meshes
- Implement sensor noise and real-world effects
- Build a complete simulated robot system
- Complete the Chapter 2 assessment project

## Adding Sensors to URDF

Sensors in Gazebo are defined as links + Gazebo plugins.

### 1. Camera Sensor

**URDF Definition:**

```xml
<!-- Camera Link -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
    <material name="red">
      <color rgba="0.8 0 0 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<!-- Mount camera on chassis -->
<joint name="camera_joint" type="fixed">
  <parent link="chassis"/>
  <child link="camera_link"/>
  <origin xyz="0.25 0 0.1" rpy="0 0 0"/>  <!-- Front of robot -->
</joint>

<!-- Gazebo Camera Plugin -->
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <update_rate>30.0</update_rate>  <!-- 30 FPS -->
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>  <!-- 80 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot</namespace>
        <argument>camera/image_raw:=camera/image_raw</argument>
        <argument>camera/camera_info:=camera/camera_info</argument>
      </ros>
      <camera_name>camera</camera_name>
      <frame_name>camera_link</frame_name>
      <hack_baseline>0.07</hack_baseline>
    </plugin>
  </sensor>
</gazebo>
```

**Viewing Camera Feed:**

```bash
# List camera topics
ros2 topic list | grep camera

# View image
ros2 run rqt_image_view rqt_image_view

# Echo camera info
ros2 topic echo /robot/camera/camera_info
```

### 2. Lidar (Laser Range Finder)

**URDF Definition:**

```xml
<!-- Lidar Link -->
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.04"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.04"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.2"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<joint name="lidar_joint" type="fixed">
  <parent link="chassis"/>
  <child link="lidar_link"/>
  <origin xyz="0 0 0.15" rpy="0 0 0"/>  <!-- Top of robot -->
</joint>

<!-- Gazebo Lidar Plugin -->
<gazebo reference="lidar_link">
  <sensor name="lidar" type="ray">
    <always_on>true</always_on>
    <update_rate>10</update_rate>
    <visualize>true</visualize>  <!-- Show rays in Gazebo -->
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>  <!-- 360 beams -->
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>  <!-- -180 degrees -->
          <max_angle>3.14159</max_angle>   <!-- +180 degrees -->
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </ray>
    <plugin name="scan_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <argument>~/out:=scan</argument>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**Visualizing Lidar in RViz:**

```bash
# RViz config
# Add → LaserScan
# Topic: /robot/scan
# Fixed Frame: odom
```

### 3. IMU (Inertial Measurement Unit)

**URDF Definition:**

```xml
<!-- IMU Link (usually at center of mass) -->
<link name="imu_link">
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
  </inertial>
</link>

<joint name="imu_joint" type="fixed">
  <parent link="chassis"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>

<!-- Gazebo IMU Plugin -->
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>  <!-- 100 Hz -->
    <plugin name="imu_controller" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <argument>~/out:=imu</argument>
      </ros>
      <frame_name>imu_link</frame_name>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>
  </sensor>
</gazebo>
```

**Reading IMU Data:**

```python
from sensor_msgs.msg import Imu

def imu_callback(self, msg: Imu):
    # Linear acceleration (m/s²)
    accel_x = msg.linear_acceleration.x
    accel_y = msg.linear_acceleration.y
    accel_z = msg.linear_acceleration.z

    # Angular velocity (rad/s)
    gyro_x = msg.angular_velocity.x
    gyro_y = msg.angular_velocity.y
    gyro_z = msg.angular_velocity.z

    # Orientation (quaternion)
    orientation = msg.orientation
```

### 4. GPS Sensor

```xml
<gazebo>
  <plugin name="gps_controller" filename="libhector_gazebo_ros_gps.so">
    <alwaysOn>true</alwaysOn>
    <updateRate>10.0</updateRate>
    <bodyName>base_link</bodyName>
    <topicName>/gps/fix</topicName>
    <velocityTopicName>/gps/fix_velocity</velocityTopicName>
    <referenceLatitude>37.4275</referenceLatitude>  <!-- Stanford, CA -->
    <referenceLongitude>-122.1697</referenceLongitude>
    <referenceAltitude>0</referenceAltitude>
    <drift>0.0 0.0 0.0</drift>
    <gaussianNoise>0.01 0.01 0.01</gaussianNoise>
  </plugin>
</gazebo>
```

## Creating Custom Gazebo Worlds

Worlds define the environment where robots operate.

### Basic World File

**`my_world.world`:**

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">

    <!-- Physics settings -->
    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Gravity -->
    <gravity>0 0 -9.81</gravity>

    <!-- Atmospheric model (optional) -->
    <atmosphere type="adiabatic"/>

    <!-- Scene settings -->
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

  </world>
</sdf>
```

### Adding Objects from Model Database

Gazebo includes pre-built models:

```xml
<!-- Add a table -->
<include>
  <uri>model://table</uri>
  <pose>2 0 0 0 0 0</pose>  <!-- x y z roll pitch yaw -->
</include>

<!-- Add a construction cone -->
<include>
  <uri>model://construction_cone</uri>
  <pose>3 1 0 0 0 0</pose>
</include>

<!-- Add a coke can -->
<include>
  <uri>model://coke_can</uri>
  <pose>2 0 0.8 0 0 0</pose>  <!-- On table -->
</include>
```

**Browse models:**
```bash
# List available models
ls /usr/share/gazebo-11/models/

# Or online: https://github.com/osrf/gazebo_models
```

### Creating Custom Models

**Custom box obstacle:**

```xml
<!-- Add to world file -->
<model name="obstacle_box">
  <static>true</static>  <!-- Doesn't move -->
  <pose>5 0 0.5 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
      <material>
        <ambient>1 0 0 1</ambient>
        <diffuse>1 0 0 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

### Custom Terrain with Heightmap

```xml
<model name="heightmap">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <heightmap>
          <uri>file://terrain.png</uri>  <!-- Grayscale image -->
          <size>100 100 10</size>  <!-- Width, length, max height -->
          <pos>0 0 -5</pos>
        </heightmap>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <heightmap>
          <uri>file://terrain.png</uri>
          <size>100 100 10</size>
          <pos>0 0 -5</pos>
          <texture>
            <diffuse>file://grass.jpg</diffuse>
            <normal>file://grass_normal.jpg</normal>
            <size>10</size>
          </texture>
        </heightmap>
      </geometry>
    </visual>
  </link>
</model>
```

### Launching Custom World

**`world_launch.py`:**

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    world_file = os.path.join(
        get_package_share_directory('my_robot_description'),
        'worlds',
        'my_world.world'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch'),
            '/gazebo.launch.py'
        ]),
        launch_arguments={'world': world_file}.items()
    )

    return LaunchDescription([gazebo])
```

## Advanced Simulation Techniques

### 1. Contact Sensors

Detect collisions:

```xml
<gazebo reference="chassis">
  <sensor name="contact_sensor" type="contact">
    <contact>
      <collision>chassis_collision</collision>
    </contact>
    <plugin name="contact_plugin" filename="libgazebo_ros_bumper.so">
      <ros>
        <namespace>/robot</namespace>
        <argument>~/out:=bumper</argument>
      </ros>
      <frame_name>chassis</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### 2. Depth Camera (RGBD)

```xml
<gazebo reference="camera_link">
  <sensor name="depth_camera" type="depth">
    <update_rate>20</update_rate>
    <camera>
      <horizontal_fov>1.047198</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.05</near>
        <far>3</far>
      </clip>
    </camera>
    <plugin name="depth_camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot</namespace>
      </ros>
      <camera_name>depth_camera</camera_name>
      <frame_name>camera_link</frame_name>
      <min_depth>0.1</min_depth>
      <max_depth>10.0</max_depth>
    </plugin>
  </sensor>
</gazebo>
```

### 3. Realistic Friction and Contact

```xml
<gazebo reference="left_wheel">
  <mu1>1.0</mu1>   <!-- Friction coefficient 1 -->
  <mu2>1.0</mu2>   <!-- Friction coefficient 2 -->
  <kp>1000000.0</kp>  <!-- Contact stiffness -->
  <kd>100.0</kd>      <!-- Contact damping -->
  <slip1>0.0</slip1>  <!-- Slip compliance -->
  <slip2>0.0</slip2>
  <fdir1>1 0 0</fdir1>  <!-- Friction direction -->
</gazebo>
```

### 4. Actor Models (Moving Pedestrians)

```xml
<actor name="actor1">
  <skin>
    <filename>walk.dae</filename>
  </skin>
  <animation name="walking">
    <filename>walk.dae</filename>
    <interpolate_x>true</interpolate_x>
  </animation>
  <script>
    <trajectory id="0" type="walking">
      <waypoint>
        <time>0</time>
        <pose>0 2 0 0 0 0</pose>
      </waypoint>
      <waypoint>
        <time>10</time>
        <pose>10 2 0 0 0 0</pose>
      </waypoint>
    </trajectory>
  </script>
</actor>
```

## Sensor Fusion Example

Combining camera and lidar data:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np


class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion')

        self.bridge = CvBridge()
        self.latest_scan = None

        # Subscribe to camera and lidar
        self.create_subscription(Image, '/robot/camera/image_raw', self.image_callback, 10)
        self.create_subscription(LaserScan, '/robot/scan', self.scan_callback, 10)

        self.get_logger().info('Sensor fusion node started')

    def scan_callback(self, msg: LaserScan):
        """Store latest lidar scan."""
        self.latest_scan = msg

    def image_callback(self, msg: Image):
        """Overlay lidar data on camera image."""
        if self.latest_scan is None:
            return

        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Find closest obstacle
        min_dist = min(self.latest_scan.ranges)
        min_idx = self.latest_scan.ranges.index(min_dist)
        angle = self.latest_scan.angle_min + min_idx * self.latest_scan.angle_increment

        # Draw on image
        text = f'Closest obstacle: {min_dist:.2f}m at {np.degrees(angle):.1f}°'
        cv2.putText(cv_image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 0), 2)

        # Display
        cv2.imshow('Sensor Fusion', cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Chapter 2 Assessment Project

**Task**: Build a warehouse inspection robot

**Requirements:**

1. **Robot URDF** (20 points):
   - Differential drive mobile base
   - Camera (front-facing) and lidar (360°)
   - IMU for odometry correction
   - Custom mesh for chassis (optional bonus)

2. **Custom Gazebo World** (20 points):
   - Warehouse environment with shelves/obstacles
   - Minimum 10 objects/obstacles
   - Textured floor and walls
   - Proper lighting

3. **Autonomous Navigation** (30 points):
   - Obstacle avoidance using lidar
   - Waypoint following (visit 3 locations)
   - Stop at each waypoint, capture image
   - Return to start position

4. **Sensor Integration** (20 points):
   - Publish camera feed at 10 Hz
   - Publish lidar scans at 5 Hz
   - Log IMU data to file
   - Visualize all sensors in RViz

5. **Documentation** (10 points):
   - README with setup instructions
   - Architecture diagram (sensors, nodes, topics)
   - Demo video (5 minutes max)

**Deliverables:**
- GitHub repository with complete ROS 2 package
- Launch file for full system
- RViz config file
- Demo video showing autonomous operation

**Bonus** (+10 points):
- SLAM mapping (use `slam_toolbox` or `cartographer`)
- Object detection with camera (use pre-trained model)
- Multi-robot simulation (2+ robots)

## Troubleshooting

### Issue: Sensor data not publishing
```bash
# Check Gazebo plugin loaded
ros2 topic list | grep /robot

# Check for errors
gz log -d
```

### Issue: Robot unstable/vibrating
**Fix**: Increase physics step size or add damping to joints

### Issue: Lidar shows no data
**Fix**: Check `visualize` tag, verify `min_range` and `max_range`

### Issue: Camera shows black screen
**Fix**: Ensure proper lighting in world, check clip near/far values

## Resources

- [Gazebo Sensors](https://classic.gazebosim.org/tutorials?cat=sensors)
- [Gazebo ROS 2 Control](https://control.ros.org/master/doc/gazebo_ros2_control/doc/index.html)
- [URDF Gazebo Extensions](https://classic.gazebosim.org/tutorials?tut=ros_urdf)
- [SDF Format Specification](http://sdformat.org/spec)
- [cv_bridge Tutorial](https://github.com/ros-perception/vision_opencv/tree/ros2/cv_bridge)

## Next Steps

Congratulations on completing Chapter 2! You now have solid robot modeling and simulation skills.

**What's next:**
- Complete Chapter 2 assessment project
- Review Gazebo concepts as needed
- Prepare for Chapter 3: [NVIDIA Isaac Platform](../03-isaac/index.md)

Chapter 3 will introduce GPU-accelerated simulation, photorealistic rendering, and synthetic data generation for AI/ML training!

---

## 📝 Weekly Quiz

Test your understanding of this week's content! The quiz is multiple choice, auto-scored, and you have 2 attempts.

**[Take the Week 7 Quiz →](/quiz?week=7)**
