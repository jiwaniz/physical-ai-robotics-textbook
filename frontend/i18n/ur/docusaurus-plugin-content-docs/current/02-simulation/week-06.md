# ہفتہ 6: URDF کے ساتھ روبوٹ ماڈلنگ اور Gazebo کی بنیادیں

## جائزہ

یہ ہفتہ URDF (Unified Robot Description Format) اور Gazebo Classic کے ساتھ روبوٹ سمیولیشن متعارف کراتا ہے۔ آپ سیکھیں گے کہ URDF میں روبوٹ کی جیومیٹری، فزکس، اور سینسرز کو کیسے بیان کریں، RViz میں روبوٹس کو کیسے visualize کریں، اور Gazebo کے فزکس انجن میں ان کی سمیولیشن کیسے کریں۔

## سیکھنے کے مقاصد

اس ہفتے کے اختتام تک، آپ یہ قابل ہوں گے:

- روبوٹ بیان کے لیے URDF فارمیٹ کو سمجھنا
- links، joints، اور visual/collision جیومیٹری کے ساتھ روبوٹ ماڈلز بنانا
- فزکس خصوصیات شامل کرنا (mass، inertia، friction)
- روبوٹ ماڈلز میں سینسرز (کیمرے، lidar، IMU) کا انضمام کرنا
- RViz میں روبوٹس کو visualize کرنا
- Gazebo Classic میں روبوٹس کی سمیولیشن کرنا
- ROS 2 topics کے ذریعے simulated روبوٹس کو کنٹرول کرنا

## روبوٹ سمیولیشن کیوں؟

مہنگے ہارڈویئر پر کوڈ تعینات کرنے سے پہلے، سمیولیشن یہ فراہم کرتی ہے:

**فوائد:**
- **حفاظت**: خطرناک منظرناموں (گرنا، ٹکراؤ) کو خطرے کے بغیر جانچیں
- **رفتار**: حقیقی وقت کی ہارڈویئر جانچ سے تیزی سے iteration کریں
- **لاگت**: ہارڈویئر کی خرابی یا ٹوٹ پھوٹ نہیں
- **تکرار پذیری**: debugging کے لیے بالکل ویسے ہی حالات
- **متوازی جانچ**: بیک وقت متعدد سمیولیشنز چلائیں
- **ڈیٹا جنریشن**: ML تربیت کے لیے مصنوعی datasets

**حدود:**
- **Sim-to-real gap**: فزکس approximations حقیقت سے مختلف ہیں
- **سینسر ماڈلنگ**: کیمرے، lidar مثالی رویہ رکھتے ہیں
- **Contact dynamics**: Friction، deformation، grasping آسان بنائے گئے ہیں
- **کمپیوٹیشنل لاگت**: اعلیٰ معیار کی سمیولیشن کو GPU کی ضرورت ہے

## URDF: Unified Robot Description Format

URDF روبوٹ kinematics، dynamics، اور visualization بیان کرنے کے لیے ایک XML فارمیٹ ہے۔

### URDF ساخت

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

### Links: روبوٹ کے اجزاء

ایک **link** ایک rigid body (chassis، wheel، arm segment) کی نمائندگی کرتا ہے۔

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

**جیومیٹری primitives:**
- `<box size="x y z"/>` - مستطیل box
- `<cylinder radius="r" length="l"/>` - سلنڈر
- `<sphere radius="r"/>` - کرہ
- `<mesh filename="model.dae"/>` - 3D mesh فائل

### Joints: Links کو جوڑنا

Joints وضاحت کرتے ہیں کہ links ایک دوسرے کی نسبت کیسے حرکت کرتے ہیں۔

**Joint کی اقسام:**

| قسم | DOF | تفصیل | مثال |
|------|-----|-------------|---------|
| **fixed** | 0 | کوئی حرکت نہیں | Camera mount |
| **revolute** | 1 | حدود کے ساتھ گردش | Robot arm joint |
| **continuous** | 1 | لامحدود گردش | Wheel |
| **prismatic** | 1 | خطی حرکت | Elevator، gripper |
| **planar** | 2 | 2D حرکت | Mobile base (x, y) |
| **floating** | 6 | آزاد حرکت | Drone |

**مثال: Revolute joint (روبوٹ بازو)**

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

**مثال: Continuous joint (پہیہ)**

```xml
<joint name="left_wheel_joint" type="continuous">
  <parent link="chassis"/>
  <child link="left_wheel"/>
  <origin xyz="-0.1 0.2 0" rpy="1.57 0 0"/>  <!-- Rotate 90° to align -->
  <axis xyz="0 0 1"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

### Inertia کی گنتی

بنیادی shapes کے لیے، یہ فارمولے استعمال کریں:

**Box (چوڑائی w، گہرائی d، اونچائی h، mass m):**
```
Ixx = (1/12) * m * (d² + h²)
Iyy = (1/12) * m * (w² + h²)
Izz = (1/12) * m * (w² + d²)
```

**Cylinder (رداس r، لمبائی l، mass m):**
```
Ixx = Iyy = (1/12) * m * (3r² + l²)
Izz = (1/2) * m * r²
```

**Sphere (رداس r، mass m):**
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

## Differential Drive روبوٹ بنانا

آئیے caster کے ساتھ مکمل 2-پہیوں والا روبوٹ بنائیں۔

### روبوٹ ڈیزائن

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

### مکمل URDF

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

## RViz میں Visualizing

RViz، ROS 2 کا 3D visualization ٹول ہے۔

### مرحلہ 1: Joint State Publisher انسٹال کریں

```bash
sudo apt install ros-humble-joint-state-publisher-gui -y
```

### مرحلہ 2: Launch فائل بنائیں

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

### مرحلہ 3: RViz کو Launch اور Configure کریں

```bash
ros2 launch my_robot_description urdf_visualize.launch.py
```

**RViz میں:**
1. **Fixed Frame** کو `base_link` پر سیٹ کریں
2. **Add** → **RobotModel** پر کلک کریں
3. Joint State Publisher GUI میں joint sliders کو منتقل کریں
4. روبوٹ ظاہر ہونا چاہیے اور حرکت کرنی چاہیے!

## Gazebo Classic انضمام

Gazebo فزکس، سینسرز، اور actuators کی سمیولیشن کرتا ہے۔

### Gazebo-مخصوص Tags شامل کرنا

Gazebo کو materials اور physics کے لیے URDF میں اضافی XML tags کی ضرورت ہے:

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

**عام Gazebo materials:**
- `Gazebo/Red`, `Gazebo/Blue`, `Gazebo/Green`
- `Gazebo/Black`, `Gazebo/White`, `Gazebo/Grey`
- `Gazebo/Orange`, `Gazebo/Yellow`

### Differential Drive Plugin

روبوٹ کو کنٹرول کرنے کے لیے، ایک Gazebo plugin شامل کریں:

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

### Gazebo کو Launch کرنا

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

### سمیولیشن چلانا

```bash
# Launch Gazebo with robot
ros2 launch my_robot_description gazebo_launch.py

# Control robot (separate terminal)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Or publish directly
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.2}}"
```

## عام URDF مسائل

### مسئلہ 1: روبوٹ زمین سے گزر جاتا ہے
**وجہ**: کوئی collision geometry نہیں یا غلط inertia
**حل**: visual geometry سے مماثل `<collision>` tags شامل کریں

### مسئلہ 2: روبوٹ پھٹ جاتا ہے/شدت سے کانپتا ہے
**وجہ**: Overlapping collision geometries یا صفر inertia
**حل**: یقینی بنائیں کہ collision shapes overlap نہ ہوں، حقیقت پسندانہ mass/inertia شامل کریں

### مسئلہ 3: پہیے گھومتے نہیں
**وجہ**: Joint axis غلط ہے یا plugin load نہیں ہوا
**حل**: Joint axis کی سمت چیک کریں (عام طور پر پہیوں کے لیے `xyz="0 0 1"`)

### مسئلہ 4: روبوٹ Gazebo میں حرکت نہیں کرتا
**وجہ**: Plugin load نہیں ہوا یا topic mismatch
**حل**: URDF میں plugin کی تصدیق کریں، `ros2 topic list` چیک کریں

## ہفتہ 6 عملی پروجیکٹ

**کام**: کیمرے کے ساتھ 4-پہیوں والا rover بنائیں

**ضروریات:**
1. URDF chassis، 4 wheels (continuous joints)، camera link کے ساتھ
2. تمام links کے لیے مناسب mass اور inertia
3. Gazebo materials اور friction coefficients
4. Differential drive plugin (2 wheels + 2 passive کے طور پر سمجھیں)
5. Camera sensor plugin (اگلا حصہ)
6. RViz visualization کے لیے launch فائل
7. Gazebo simulation کے لیے launch فائل
8. Gazebo میں روبوٹ کی حرکت دکھاتی Demo ویڈیو

## وسائل

- [URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- [Gazebo Classic Documentation](https://classic.gazebosim.org/)
- [Gazebo ROS 2 Plugins](https://github.com/ros-simulation/gazebo_ros_pkgs)
- [URDF Validator](http://wiki.ros.org/urdf/Tutorials/Check%20URDF)
- [SolidWorks to URDF](http://wiki.ros.org/sw_urdf_exporter)

## اگلے مراحل

بہترین کام! اب آپ جانتے ہیں کہ URDF میں روبوٹس کو کیسے model کریں اور Gazebo میں ان کی سمیولیشن کیسے کریں۔

اگلا ہفتہ: [ہفتہ 7: Sensors، Worlds، اور Advanced Gazebo](week-07.md)

ہم سینسرز (کیمرے، lidar) شامل کریں گے، حسب ضرورت worlds بنائیں گے، اور اعلیٰ درجے کی سمیولیشن تکنیک تلاش کریں گے!

---

## 📝 ہفتہ وار کوئز

اس ہفتے کے مواد کی اپنی سمجھ جانچیں! کوئز کثیر الانتخاب (multiple choice) ہے، خودکار طور پر اسکور کیا جاتا ہے، اور آپ کے پاس 2 کوششیں ہیں۔

**[ہفتہ 6 کوئز لیں →](/quiz?week=6)**
