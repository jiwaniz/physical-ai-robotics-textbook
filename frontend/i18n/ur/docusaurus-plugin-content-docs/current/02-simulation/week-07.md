# ہفتہ 7: Sensors، Worlds، اور Advanced Gazebo

## جائزہ

یہ ہفتہ سینسر انضمام، حسب ضرورت world کی تخلیق، اور اعلیٰ درجے کی سمیولیشن تکنیک کے ساتھ آپ کی Gazebo مہارت کو بڑھاتا ہے۔ آپ اپنے روبوٹ میں کیمرے، lidar، اور IMUs شامل کریں گے، حقیقت پسندانہ ماحول بنائیں گے، اور باب 2 کے تشخیصی پروجیکٹ کو مکمل کریں گے۔

## سیکھنے کے مقاصد

اس ہفتے کے اختتام تک، آپ یہ قابل ہوں گے:

- روبوٹ URDF میں سینسرز (camera، lidar، IMU، GPS) کا انضمام کرنا
- ROS 2 topics کے ذریعے سینسر ڈیٹا پر کارروائی کرنا
- ماڈلز اور terrain کے ساتھ حسب ضرورت Gazebo worlds بنانا
- Gazebo model database استعمال کرنا اور حسب ضرورت meshes import کرنا
- سینسر noise اور حقیقی دنیا کے اثرات کو implement کرنا
- مکمل simulated روبوٹ سسٹم بنانا
- باب 2 کے تشخیصی پروجیکٹ کو مکمل کرنا

## URDF میں Sensors شامل کرنا

Gazebo میں سینسرز کو links + Gazebo plugins کے طور پر بیان کیا جاتا ہے۔

### 1. Camera Sensor

**URDF تعریف:**

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

**Camera Feed دیکھنا:**

```bash
# List camera topics
ros2 topic list | grep camera

# View image
ros2 run rqt_image_view rqt_image_view

# Echo camera info
ros2 topic echo /robot/camera/camera_info
```

### 2. Lidar (Laser Range Finder)

**URDF تعریف:**

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

**RViz میں Lidar کو Visualizing:**

```bash
# RViz config
# Add → LaserScan
# Topic: /robot/scan
# Fixed Frame: odom
```

### 3. IMU (Inertial Measurement Unit)

**URDF تعریف:**

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

**IMU ڈیٹا پڑھنا:**

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

## حسب ضرورت Gazebo Worlds بنانا

Worlds اس ماحول کی وضاحت کرتے ہیں جہاں روبوٹس کام کرتے ہیں۔

### بنیادی World فائل

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

### Model Database سے Objects شامل کرنا

Gazebo میں پہلے سے بنے ہوئے ماڈلز شامل ہیں:

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

**ماڈلز براؤز کریں:**
```bash
# List available models
ls /usr/share/gazebo-11/models/

# Or online: https://github.com/osrf/gazebo_models
```

### حسب ضرورت Models بنانا

**حسب ضرورت box رکاوٹ:**

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

### Heightmap کے ساتھ حسب ضرورت Terrain

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

### حسب ضرورت World کو Launch کرنا

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

## اعلیٰ درجے کی سمیولیشن تکنیک

### 1. Contact Sensors

ٹکراؤ کا پتہ لگائیں:

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

### 3. حقیقت پسندانہ Friction اور Contact

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

### 4. Actor Models (حرکت کرتے پیدل چلنے والے)

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

## Sensor Fusion مثال

کیمرا اور lidar ڈیٹا کو ملانا:

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

## باب 2 تشخیصی پروجیکٹ

**کام**: گودام معائنہ روبوٹ بنائیں

**ضروریات:**

1. **روبوٹ URDF** (20 پوائنٹس):
   - Differential drive mobile base
   - کیمرہ (سامنے کی طرف) اور lidar (360°)
   - Odometry اصلاح کے لیے IMU
   - Chassis کے لیے حسب ضرورت mesh (اختیاری بونس)

2. **حسب ضرورت Gazebo World** (20 پوائنٹس):
   - شیلفوں/رکاوٹوں کے ساتھ گودام کا ماحول
   - کم از کم 10 اشیاء/رکاوٹیں
   - Textured فرش اور دیواریں
   - مناسب روشنی

3. **خودکار نیویگیشن** (30 پوائنٹس):
   - Lidar استعمال کرتے ہوئے رکاوٹوں سے بچنا
   - Waypoint following (3 مقامات کا دورہ کریں)
   - ہر waypoint پر رکیں، تصویر لیں
   - شروعاتی مقام پر واپس آئیں

4. **سینسر انضمام** (20 پوائنٹس):
   - 10 Hz پر camera feed publish کریں
   - 5 Hz پر lidar scans publish کریں
   - IMU ڈیٹا کو فائل میں log کریں
   - RViz میں تمام سینسرز کو visualize کریں

5. **دستاویزات** (10 پوائنٹس):
   - سیٹ اپ ہدایات کے ساتھ README
   - فن تعمیر کا خاکہ (سینسرز، nodes، topics)
   - Demo ویڈیو (زیادہ سے زیادہ 5 منٹ)

**فراہم کی جانے والی چیزیں:**
- مکمل ROS 2 پیکیج کے ساتھ GitHub repository
- مکمل سسٹم کے لیے launch فائل
- RViz config فائل
- خودکار آپریشن دکھاتی Demo ویڈیو

**بونس** (+10 پوائنٹس):
- SLAM mapping (`slam_toolbox` یا `cartographer` استعمال کریں)
- کیمرے کے ساتھ object detection (پہلے سے تربیت یافتہ ماڈل استعمال کریں)
- Multi-robot simulation (2+ روبوٹس)

## مسائل کا حل

### مسئلہ: سینسر ڈیٹا publish نہیں ہو رہا
```bash
# Check Gazebo plugin loaded
ros2 topic list | grep /robot

# Check for errors
gz log -d
```

### مسئلہ: روبوٹ غیر مستحکم/کانپ رہا ہے
**حل**: Physics step size بڑھائیں یا joints میں damping شامل کریں

### مسئلہ: Lidar کوئی ڈیٹا نہیں دکھاتا
**حل**: `visualize` tag چیک کریں، `min_range` اور `max_range` کی تصدیق کریں

### مسئلہ: کیمرا سیاہ اسکرین دکھاتا ہے
**حل**: World میں مناسب روشنی کو یقینی بنائیں، clip near/far values چیک کریں

## وسائل

- [Gazebo Sensors](https://classic.gazebosim.org/tutorials?cat=sensors)
- [Gazebo ROS 2 Control](https://control.ros.org/master/doc/gazebo_ros2_control/doc/index.html)
- [URDF Gazebo Extensions](https://classic.gazebosim.org/tutorials?tut=ros_urdf)
- [SDF Format Specification](http://sdformat.org/spec)
- [cv_bridge Tutorial](https://github.com/ros-perception/vision_opencv/tree/ros2/cv_bridge)

## اگلے مراحل

باب 2 مکمل کرنے پر مبارکباد! اب آپ کے پاس ٹھوس روبوٹ ماڈلنگ اور سمیولیشن مہارتیں ہیں۔

**آگے کیا ہے:**
- باب 2 کے تشخیصی پروجیکٹ کو مکمل کریں
- ضرورت کے مطابق Gazebo تصورات کا جائزہ لیں
- باب 3 کے لیے تیاری کریں: [NVIDIA Isaac پلیٹ فارم](../03-isaac/index.md)

باب 3 GPU-accelerated سمیولیشن، photorealistic رینڈرنگ، اور AI/ML تربیت کے لیے مصنوعی ڈیٹا جنریشن متعارف کرائے گا!

---

## 📝 ہفتہ وار کوئز

اس ہفتے کے مواد کی اپنی سمجھ جانچیں! کوئز کثیر الانتخاب (multiple choice) ہے، خودکار طور پر اسکور کیا جاتا ہے، اور آپ کے پاس 2 کوششیں ہیں۔

**[ہفتہ 7 کوئز لیں →](/quiz?week=7)**
