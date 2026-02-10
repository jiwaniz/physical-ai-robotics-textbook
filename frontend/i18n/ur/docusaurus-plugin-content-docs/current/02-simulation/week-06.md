# ہفتہ 6: یو آر ڈی ایف کے ساتھ روبوٹ ماڈلنگ اور گزیبو کی بنیادیں

## جائزہ

یہ ہفتہ یو آر ڈی ایف (یونیفائیڈ روبوٹ ڈسکرپشن فارمیٹ) اور گزیبو کلاسک کے ساتھ روبوٹ سمیولیشن متعارف کراتا ہے۔ آپ سیکھیں گے کہ یو آر ڈی ایف میں روبوٹ کی جیومیٹری، فزکس، اور سینسرز کو کیسے بیان کریں، آر ویز میں روبوٹس کو کیسے ویژولائز کریں، اور گزیبو کے فزکس انجن میں ان کی سمیولیشن کیسے کریں۔

## سیکھنے کے مقاصد

اس ہفتے کے اختتام تک، آپ یہ قابل ہوں گے:

- روبوٹ بیان کے لیے یو آر ڈی ایف فارمیٹ کو سمجھنا
- لنکس، جوائنٹس، اور ویژول/کولیژن جیومیٹری کے ساتھ روبوٹ ماڈلز بنانا
- فزکس خصوصیات شامل کرنا (ماس، انرشیا، فرکشن)
- روبوٹ ماڈلز میں سینسرز (کیمرے، لائیڈار، آئی ایم یو) کا انضمام کرنا
- آر ویز میں روبوٹس کو ویژولائز کرنا
- گزیبو کلاسک میں روبوٹس کی سمیولیشن کرنا
- آر او ایس 2 ٹاپکس کے ذریعے سمیولیٹڈ روبوٹس کو کنٹرول کرنا

## روبوٹ سمیولیشن کیوں؟

مہنگے ہارڈویئر پر کوڈ تعینات کرنے سے پہلے، سمیولیشن یہ فراہم کرتی ہے:

**فوائد:**
- **حفاظت**: خطرناک منظرناموں (گرنا، ٹکراؤ) کو خطرے کے بغیر جانچیں
- **رفتار**: حقیقی وقت کی ہارڈویئر جانچ سے تیزی سے اٹریشن کریں
- **لاگت**: ہارڈویئر کی خرابی یا ٹوٹ پھوٹ نہیں
- **تکرار پذیری**: ڈیبگنگ کے لیے بالکل ویسے ہی حالات
- **متوازی جانچ**: بیک وقت متعدد سمیولیشنز چلائیں
- **ڈیٹا جنریشن**: ایم ایل ٹریننگ کے لیے سنتھیٹک ڈیٹاسیٹس

**حدود:**
- **سم-ٹو-ریئل گیپ**: فزکس اپروکسمیشنز حقیقت سے مختلف ہیں
- **سینسر ماڈلنگ**: کیمرے، لائیڈار مثالی رویہ رکھتے ہیں
- **کانٹیکٹ ڈائنامکس**: فرکشن، ڈیفارمیشن، گراسپنگ آسان بنائے گئے ہیں
- **کمپیوٹیشنل لاگت**: اعلیٰ معیار کی سمیولیشن کو جی پی یو کی ضرورت ہے

## یو آر ڈی ایف: یونیفائیڈ روبوٹ ڈسکرپشن فارمیٹ

یو آر ڈی ایف روبوٹ کائنامیٹکس، ڈائنامکس، اور ویژولائزیشن بیان کرنے کے لیے ایک ایکس ایم ایل فارمیٹ ہے۔

### یو آر ڈی ایف ساخت

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

### لنکس: روبوٹ کے اجزاء

ایک **لنک** ایک رجڈ باڈی (چیسس، وہیل، آرم سیگمنٹ) کی نمائندگی کرتا ہے۔

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

**جیومیٹری پریمیٹوز:**
- `<box size="x y z"/>` - مستطیل باکس
- `<cylinder radius="r" length="l"/>` - سلنڈر
- `<sphere radius="r"/>` - کرہ
- `<mesh filename="model.dae"/>` - 3ڈی میش فائل

### جوائنٹس: لنکس کو جوڑنا

جوائنٹس وضاحت کرتے ہیں کہ لنکس ایک دوسرے کی نسبت کیسے حرکت کرتے ہیں۔

**جوائنٹ کی اقسام:**

| قسم | ڈی او ایف | تفصیل | مثال |
|------|-----|-------------|---------|
| **فکسڈ** | 0 | کوئی حرکت نہیں | کیمرا ماؤنٹ |
| **ریوولیوٹ** | 1 | حدود کے ساتھ گردش | روبوٹ آرم جوائنٹ |
| **کنٹینیوس** | 1 | لامحدود گردش | پہیہ |
| **پرزمیٹک** | 1 | خطی حرکت | ایلیویٹر، گرپر |
| **پلینر** | 2 | 2ڈی حرکت | موبائل بیس |
| **فلوٹنگ** | 6 | آزاد حرکت | ڈرون |

**مثال: ریوولیوٹ جوائنٹ (روبوٹ بازو)**

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

**مثال: کنٹینیوس جوائنٹ (پہیہ)**

```xml
<joint name="left_wheel_joint" type="continuous">
  <parent link="chassis"/>
  <child link="left_wheel"/>
  <origin xyz="-0.1 0.2 0" rpy="1.57 0 0"/>  <!-- Rotate 90° to align -->
  <axis xyz="0 0 1"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

### انرشیا کی گنتی

بنیادی شکلوں کے لیے، یہ فارمولے استعمال کریں:

**باکس (چوڑائی w، گہرائی d، اونچائی h، ماس m):**
```
Ixx = (1/12) * m * (d² + h²)
Iyy = (1/12) * m * (w² + h²)
Izz = (1/12) * m * (w² + d²)
```

**سلنڈر (ریڈیئس r، لمبائی l، ماس m):**
```
Ixx = Iyy = (1/12) * m * (3r² + l²)
Izz = (1/2) * m * r²
```

**اسفیئر (ریڈیئس r، ماس m):**
```
Ixx = Iyy = Izz = (2/5) * m * r²
```

**پائتھون ہیلپر:**
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

## ڈیفرینشل ڈرائیو روبوٹ بنانا

آئیے کیسٹر کے ساتھ مکمل 2 پہیوں والا روبوٹ بنائیں۔

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

### مکمل یو آر ڈی ایف

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

## آر ویز میں ویژولائزنگ

آر ویز، آر او ایس 2 کا 3ڈی ویژولائزیشن ٹول ہے۔

### مرحلہ 1: جوائنٹ اسٹیٹ پبلشر انسٹال کریں

```bash
sudo apt install ros-humble-joint-state-publisher-gui -y
```

### مرحلہ 2: لانچ فائل بنائیں

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

### مرحلہ 3: آر ویز کو لانچ اور کنفیگر کریں

```bash
ros2 launch my_robot_description urdf_visualize.launch.py
```

**آر ویز میں:**
1. **فکسڈ فریم** کو `base_link` پر سیٹ کریں
2. **ایڈ** → **روبوٹ ماڈل** پر کلک کریں
3. جوائنٹ اسٹیٹ پبلشر جی یو آئی میں جوائنٹ سلائیڈرز کو منتقل کریں
4. روبوٹ ظاہر ہونا چاہیے اور حرکت کرنی چاہیے!

## گزیبو کلاسک انضمام

گزیبو فزکس، سینسرز، اور ایکچویٹرز کی سمیولیشن کرتا ہے۔

### گزیبو مخصوص ٹیگز شامل کرنا

گزیبو کو میٹیریلز اور فزکس کے لیے یو آر ڈی ایف میں اضافی ایکس ایم ایل ٹیگز کی ضرورت ہے:

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

**عام گزیبو میٹیریلز:**
- `Gazebo/Red`, `Gazebo/Blue`, `Gazebo/Green`
- `Gazebo/Black`, `Gazebo/White`, `Gazebo/Grey`
- `Gazebo/Orange`, `Gazebo/Yellow`

### ڈیفرینشل ڈرائیو پلگ ان

روبوٹ کو کنٹرول کرنے کے لیے، ایک گزیبو پلگ ان شامل کریں:

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

### گزیبو کو لانچ کرنا

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

## عام یو آر ڈی ایف مسائل

### مسئلہ 1: روبوٹ زمین سے گزر جاتا ہے
**وجہ**: کوئی کولیژن جیومیٹری نہیں یا غلط انرشیا
**حل**: ویژول جیومیٹری سے مماثل `<collision>` ٹیگز شامل کریں

### مسئلہ 2: روبوٹ پھٹ جاتا ہے/شدت سے کانپتا ہے
**وجہ**: اوورلیپنگ کولیژن جیومیٹریز یا صفر انرشیا
**حل**: یقینی بنائیں کہ کولیژن شکلیں اوورلیپ نہ ہوں، حقیقت پسندانہ ماس/انرشیا شامل کریں

### مسئلہ 3: پہیے گھومتے نہیں
**وجہ**: جوائنٹ ایکسس غلط ہے یا پلگ ان لوڈ نہیں ہوا
**حل**: جوائنٹ ایکسس کی سمت چیک کریں (عام طور پر پہیوں کے لیے `xyz="0 0 1"`)

### مسئلہ 4: روبوٹ گزیبو میں حرکت نہیں کرتا
**وجہ**: پلگ ان لوڈ نہیں ہوا یا ٹاپک مسمیچ
**حل**: یو آر ڈی ایف میں پلگ ان کی تصدیق کریں، `ros2 topic list` چیک کریں

## ہفتہ 6 عملی پروجیکٹ

**کام**: کیمرے کے ساتھ 4 پہیوں والا روور بنائیں

**ضروریات:**
1. یو آر ڈی ایف چیسس، 4 وہیلز (کنٹینیوس جوائنٹس)، کیمرا لنک کے ساتھ
2. تمام لنکس کے لیے مناسب ماس اور انرشیا
3. گزیبو میٹیریلز اور فرکشن کوایفیشنٹس
4. ڈیفرینشل ڈرائیو پلگ ان (2 وہیلز + 2 پیسو کے طور پر سمجھیں)
5. کیمرا سینسر پلگ ان (اگلا حصہ)
6. آر ویز ویژولائزیشن کے لیے لانچ فائل
7. گزیبو سمیولیشن کے لیے لانچ فائل
8. گزیبو میں روبوٹ کی حرکت دکھاتی ڈیمو ویڈیو

## وسائل

- [یو آر ڈی ایف ٹیوٹوریلز](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- [گزیبو کلاسک ڈاکیومینٹیشن](https://classic.gazebosim.org/)
- [گزیبو آر او ایس 2 پلگ انز](https://github.com/ros-simulation/gazebo_ros_pkgs)
- [یو آر ڈی ایف ویلیڈیٹر](http://wiki.ros.org/urdf/Tutorials/Check%20URDF)
- [سالڈ ورکس سے یو آر ڈی ایف](http://wiki.ros.org/sw_urdf_exporter)

## اگلے مراحل

بہترین کام! اب آپ جانتے ہیں کہ یو آر ڈی ایف میں روبوٹس کو کیسے ماڈل کریں اور گزیبو میں ان کی سمیولیشن کیسے کریں۔

اگلا ہفتہ: [ہفتہ 7: سینسرز، ورلڈز، اور ایڈوانسڈ گزیبو](week-07.md)

ہم سینسرز (کیمرے، لائیڈار) شامل کریں گے، حسب ضرورت ورلڈز بنائیں گے، اور اعلیٰ درجے کی سمیولیشن تکنیک تلاش کریں گے!

---

## 📝 ہفتہ وار کوئز

اس ہفتے کے مواد کی اپنی سمجھ جانچیں! کوئز کثیر الانتخاب ہے، خودکار طور پر اسکور کیا جاتا ہے، اور آپ کے پاس 2 کوششیں ہیں۔

**[ہفتہ 6 کوئز لیں →](/quiz?week=6)**
