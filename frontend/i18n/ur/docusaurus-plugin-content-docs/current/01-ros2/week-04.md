# ہفتہ 4: Nodes، Topics، Publishers اور Subscribers

## جائزہ

یہ ہفتہ pub-sub (publish-subscribe) communication pattern کو گہرائی سے دریافت کرتا ہے۔ آپ topic architectures ڈیزائن کرنا، standard message types کے ساتھ کام کرنا، Quality of Service (QoS) policies کو implement کرنا، اور sensors اور actuators کے ساتھ ایک multi-node robot سسٹم بنانا سیکھیں گے۔

## سیکھنے کے مقاصد

اس ہفتے کے اختتام تک، آپ یہ کر سکیں گے:

- Multi-node سسٹمز کے لیے topic architectures ڈیزائن کرنا
- Standard ROS 2 message types استعمال کرنا (`std_msgs`، `sensor_msgs`، `geometry_msgs`)
- Custom message definitions بنانا
- Reliability کے لیے QoS (Quality of Service) policies کو configure کرنا
- مضبوط publisher اور subscriber nodes کو implement کرنا
- مکمل sensor-to-actuator pipeline بنانا

## Publish-Subscribe Pattern

### Pub-Sub کیوں؟

Publish-subscribe pattern ڈیٹا کے producers اور consumers کو **decouple** کرتا ہے:

**فوائد:**
- **Scalability**: Publishers تبدیل کیے بغیر subscribers شامل کریں
- **Modularity**: Nodes کو ایک دوسرے کے بارے میں جانने کی ضرورت نہیں
- **Flexibility**: Nodes کو آسانی سے mix اور match کریں
- **Parallel processing**: متعدد subscribers آزادانہ طور پر ڈیٹا پروسیس کرتے ہیں

**Trade-offs:**
- کوئی delivery guarantees نہیں (مناسب QoS کے بغیر)
- کوئی request-response نہیں (اس کے لیے services استعمال کریں)
- اگر subscribers سست ہیں تو ممکنہ data overload

### Pub-Sub Flow

```
┌─────────────────┐
│  Camera Driver  │ ──┐
│  (Publisher)    │   │
└─────────────────┘   │
                      ▼
                 /camera/image
                   (Topic)
                      │
        ┌─────────────┼─────────────┐
        │             │             │
        ▼             ▼             ▼
┌─────────────┐ ┌─────────────┐ ┌─────────────┐
│   Object    │ │    SLAM     │ │   Video     │
│  Detector   │ │   System    │ │  Recorder   │
│(Subscriber) │ │(Subscriber) │ │(Subscriber) │
└─────────────┘ └─────────────┘ └─────────────┘
```

## Standard Message Types

ROS 2 standard packages میں عام message types فراہم کرتا ہے۔

### 1. std_msgs: بنیادی اقسام

```python
from std_msgs.msg import String, Int32, Float64, Bool

# String message
msg = String()
msg.data = "Hello, ROS 2!"

# Numeric messages
int_msg = Int32()
int_msg.data = 42

float_msg = Float64()
float_msg.data = 3.14159
```

**عام std_msgs اقسام:**
- `String`، `Char`
- `Bool`
- `Int8`، `Int16`، `Int32`، `Int64`
- `UInt8`، `UInt16`، `UInt32`، `UInt64`
- `Float32`، `Float64`
- `Header` (timestamp + frame_id)

### 2. geometry_msgs: مقامی ڈیٹا

```python
from geometry_msgs.msg import Point, Pose, Twist, Vector3

# 3D Point
point = Point()
point.x = 1.0
point.y = 2.0
point.z = 0.5

# Twist (linear + angular velocity)
twist = Twist()
twist.linear.x = 0.5   # Move forward at 0.5 m/s
twist.angular.z = 0.2  # Rotate at 0.2 rad/s
```

**عام geometry_msgs اقسام:**
- `Point`، `Point32` (3D position)
- `Quaternion` (orientation)
- `Pose` (position + orientation)
- `PoseStamped` (Pose + Header)
- `Twist` (linear + angular velocity)
- `Transform` (translation + rotation)

### 3. sensor_msgs: سینسر ڈیٹا

```python
from sensor_msgs.msg import Image, LaserScan, Imu, JointState

# Camera image
img = Image()
img.header.stamp = self.get_clock().now().to_msg()
img.header.frame_id = 'camera_frame'
img.height = 480
img.width = 640
img.encoding = 'rgb8'
img.data = [...]  # Raw image bytes

# Lidar scan
scan = LaserScan()
scan.angle_min = -1.57  # -90 degrees
scan.angle_max = 1.57   # +90 degrees
scan.angle_increment = 0.01
scan.ranges = [...]  # Distance measurements
```

**عام sensor_msgs اقسام:**
- `Image`، `CompressedImage`
- `CameraInfo`
- `LaserScan`، `PointCloud2`
- `Imu` (inertial measurement unit)
- `JointState` (robot joint positions/velocities)
- `NavSatFix` (GPS)

### 4. nav_msgs: نیویگیشن ڈیٹا

```python
from nav_msgs.msg import Odometry, Path

# Robot odometry
odom = Odometry()
odom.header.stamp = self.get_clock().now().to_msg()
odom.header.frame_id = 'odom'
odom.child_frame_id = 'base_link'
odom.pose.pose.position.x = 1.5
odom.pose.pose.position.y = 2.0
odom.twist.twist.linear.x = 0.3
```

**عام nav_msgs اقسام:**
- `Odometry` (robot pose + velocity)
- `Path` (poses کی تسلسل)
- `GridCells` (occupancy grid cells)

## Custom Messages بنانا

جب standard messages آپ کی ضروریات کے مطابق نہ ہوں، تو اپنی custom بنائیں۔

### قدم 1: Message متعین کریں

`msg/RobotStatus.msg` بنائیں:
```
# RobotStatus.msg - Custom message for robot state

std_msgs/Header header
string robot_name
float64 battery_level      # Percentage (0-100)
float64 temperature        # Celsius
geometry_msgs/Point position
bool is_moving
string current_task
```

### قدم 2: package.xml کو اپ ڈیٹ کریں

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

### قدم 3: CMakeLists.txt (C++) یا setup.py (Python) کو اپ ڈیٹ کریں

**ament_cmake کے لیے:**
```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs
)
```

**ament_python کے لیے:**
آپ کو ایک الگ interface package کی ضرورت ہے (تجویز کردہ pattern: `my_pkg_interfaces`)۔

### قدم 4: Build اور استعمال کریں

```bash
colcon build --packages-select my_robot_interfaces
source install/setup.bash

# Verify message
ros2 interface show my_robot_interfaces/msg/RobotStatus
```

```python
from my_robot_interfaces.msg import RobotStatus

msg = RobotStatus()
msg.robot_name = "PhysicsBot-001"
msg.battery_level = 87.5
msg.temperature = 42.3
msg.is_moving = True
```

## Quality of Service (QoS)

QoS policies کنٹرول کرتی ہیں کہ publishers اور subscribers کے درمیان messages کیسے deliver ہوتے ہیں۔

### QoS Parameters

| Policy | اختیارات | تفصیل |
|--------|---------|-------------|
| **Reliability** | `RELIABLE`، `BEST_EFFORT` | Delivery کی ضمانت دیں یا packet loss کی اجازت دیں |
| **Durability** | `VOLATILE`، `TRANSIENT_LOCAL` | دیر سے شامل ہونے والے subscribers کے لیے messages رکھیں |
| **History** | `KEEP_LAST(n)`، `KEEP_ALL` | Messages کے لیے queue size |
| **Deadline** | Duration | Messages کے درمیان زیادہ سے زیادہ وقت |
| **Lifespan** | Duration | Message کے valid ہونے کا زیادہ سے زیادہ وقت |
| **Liveliness** | `AUTOMATIC`، `MANUAL` | مردہ publishers کا پتہ لگائیں |

### عام QoS Profiles

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

# Sensor data (tolerate loss, low latency)
sensor_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)

# Control commands (must arrive, order matters)
control_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)

# Configuration data (late-joiners need it)
config_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)

# Create publisher with QoS
self.publisher_ = self.create_publisher(
    Twist,
    '/cmd_vel',
    control_qos  # Use control QoS
)
```

### QoS Compatibility

Publishers اور subscribers کے پاس **compatible** QoS ہونا ضروری ہے:

| Publisher | Subscriber | Compatible؟ |
|-----------|------------|-------------|
| RELIABLE | RELIABLE | ✅ ہاں |
| RELIABLE | BEST_EFFORT | ✅ ہاں (subscriber کو best effort ملتا ہے) |
| BEST_EFFORT | RELIABLE | ❌ نہیں |
| BEST_EFFORT | BEST_EFFORT | ✅ ہاں |

**اصول:**
- Sensors → `BEST_EFFORT` (reliability سے زیادہ رفتار)
- Control → `RELIABLE` (commands نہیں کھونے چاہیئں)
- Status → `RELIABLE` (robot کی state جاننا ضروری ہے)

## Multi-Node Robot سسٹم بنانا

آئیے sensor اور motor nodes کے ساتھ ایک سادہ mobile robot بنائیں۔

### سسٹم آرکیٹیکچر

```
┌──────────────┐     /sensor/distance     ┌──────────────┐
│   Distance   │─────────────────────────▶│  Obstacle    │
│    Sensor    │   (Float64, 10 Hz)       │  Detector    │
└──────────────┘                          └──────┬───────┘
                                                 │
                                                 │ /motor/cmd
                                                 │ (Twist)
                                                 ▼
                                          ┌──────────────┐
                                          │    Motor     │
                                          │  Controller  │
                                          └──────────────┘
```

### Node 1: Distance Sensor Simulator

**`distance_sensor_node.py`:**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random


class DistanceSensorNode(Node):
    """
    Simulates a distance sensor publishing range measurements.
    """

    def __init__(self):
        super().__init__('distance_sensor')

        # Declare parameters
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('min_range', 0.1)      # meters
        self.declare_parameter('max_range', 5.0)      # meters

        # Get parameters
        rate = self.get_parameter('publish_rate').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value

        # Create publisher with sensor QoS
        from rclpy.qos import qos_profile_sensor_data
        self.publisher_ = self.create_publisher(
            Float64,
            '/sensor/distance',
            qos_profile_sensor_data
        )

        # Create timer
        timer_period = 1.0 / rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # State
        self.obstacle_distance = 2.0  # Start at 2m

        self.get_logger().info(
            f'Distance sensor started (rate: {rate} Hz, range: {self.min_range}-{self.max_range}m)'
        )

    def timer_callback(self):
        """Simulate sensor reading and publish."""
        # Simulate obstacle moving closer/farther (random walk)
        self.obstacle_distance += random.uniform(-0.1, 0.1)
        self.obstacle_distance = max(self.min_range, min(self.max_range, self.obstacle_distance))

        # Create and publish message
        msg = Float64()
        msg.data = self.obstacle_distance
        self.publisher_.publish(msg)

        self.get_logger().debug(f'Distance: {msg.data:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    node = DistanceSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Node 2: Obstacle Detector

**`obstacle_detector_node.py`:**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class ObstacleDetectorNode(Node):
    """
    Detects obstacles and sends motor commands to avoid them.
    """

    def __init__(self):
        super().__init__('obstacle_detector')

        # Parameters
        self.declare_parameter('safety_distance', 0.5)  # meters
        self.declare_parameter('max_speed', 0.5)        # m/s

        self.safety_distance = self.get_parameter('safety_distance').value
        self.max_speed = self.get_parameter('max_speed').value

        # Subscriber (sensor QoS)
        from rclpy.qos import qos_profile_sensor_data
        self.subscription = self.create_subscription(
            Float64,
            '/sensor/distance',
            self.distance_callback,
            qos_profile_sensor_data
        )

        # Publisher (reliable QoS for control)
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy
        control_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=10
        )
        self.cmd_publisher = self.create_publisher(
            Twist,
            '/motor/cmd',
            control_qos
        )

        self.get_logger().info(
            f'Obstacle detector started (safety: {self.safety_distance}m, max speed: {self.max_speed}m/s)'
        )

    def distance_callback(self, msg):
        """Process distance reading and send motor command."""
        distance = msg.data

        # Create velocity command
        cmd = Twist()

        if distance < self.safety_distance:
            # Obstacle too close - stop and turn
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Rotate to avoid
            self.get_logger().warn(f'Obstacle detected at {distance:.2f}m - avoiding!')
        elif distance < self.safety_distance * 2:
            # Obstacle approaching - slow down
            cmd.linear.x = self.max_speed * 0.3
            cmd.angular.z = 0.0
            self.get_logger().info(f'Slowing down (distance: {distance:.2f}m)')
        else:
            # Clear path - full speed
            cmd.linear.x = self.max_speed
            cmd.angular.z = 0.0

        # Publish command
        self.cmd_publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Node 3: Motor Controller

**`motor_controller_node.py`:**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MotorControllerNode(Node):
    """
    Receives velocity commands and controls motors (simulated).
    """

    def __init__(self):
        super().__init__('motor_controller')

        # Subscriber
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy
        control_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=10
        )
        self.subscription = self.create_subscription(
            Twist,
            '/motor/cmd',
            self.cmd_callback,
            control_qos
        )

        # State
        self.current_linear = 0.0
        self.current_angular = 0.0

        self.get_logger().info('Motor controller started')

    def cmd_callback(self, msg):
        """Execute velocity command."""
        linear = msg.linear.x
        angular = msg.angular.z

        # Simulate motor control
        if linear != self.current_linear or angular != self.current_angular:
            self.get_logger().info(
                f'Motor command: linear={linear:.2f} m/s, angular={angular:.2f} rad/s'
            )
            self.current_linear = linear
            self.current_angular = angular

            # In real robot: Send PWM signals to motors


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### سسٹم چلانا

```bash
# Terminal 1: Sensor
ros2 run my_robot distance_sensor_node

# Terminal 2: Detector
ros2 run my_robot obstacle_detector_node

# Terminal 3: Motors
ros2 run my_robot motor_controller_node

# Terminal 4: Monitor
ros2 topic echo /motor/cmd
```

## بہترین طریقے

### 1. Topic ناموں کے کنونشنز
```
/robot_name/sensor_type/data_type
/robot1/camera/image_raw
/robot1/lidar/scan
/robot1/motor/cmd_vel
```

### 2. Timestamps کے لیے Headers استعمال کریں
```python
from std_msgs.msg import Header

header = Header()
header.stamp = self.get_clock().now().to_msg()
header.frame_id = 'base_link'
```

### 3. Subscription Lifecycle کو Handle کریں
```python
def __init__(self):
    # ...
    self.last_msg_time = self.get_clock().now()
    self.create_timer(1.0, self.check_timeout)

def check_timeout(self):
    elapsed = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
    if elapsed > 2.0:  # 2 second timeout
        self.get_logger().warn('No messages received for 2 seconds!')
```

### 4. Fine-Grained Control کے لیے rclpy.spin_once استعمال کریں
```python
while rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.1)
    # Custom processing here
```

## ہفتہ 4 کا عملی پراجیکٹ

**کام**: ایک "Line Follower" simulation بنائیں

**ضروریات:**
1. **Sensor node**: Simulated line position publish کرتا ہے (-1.0 سے 1.0، بائیں سے دائیں)
2. **Controller node**: Sensor کو subscribe کرتا ہے، steering commands publish کرتا ہے
3. **Visualizer node**: Robot path کو file میں log کرتا ہے
4. Custom message `LinePosition` استعمال کریں جس میں fields ہوں: `header`، `position`، `confidence`
5. QoS policies کو مناسب طریقے سے implement کریں
6. PID controller gains کے لیے parameter tuning شامل کریں

**بونس**: تمام nodes کو بیک وقت شروع کرنے کے لیے launch file بنائیں (اگلے ہفتے covered)۔

## کوئز کے سوالات

1. `RELIABLE` اور `BEST_EFFORT` QoS میں کیا فرق ہے؟
2. `TRANSIENT_LOCAL` durability کب استعمال کرنی چاہیے؟
3. Messages میں headers کیوں اہم ہیں؟
4. آپ کیسے چیک کرتے ہیں کہ publisher اور subscriber کے پاس compatible QoS ہے؟
5. کیا ہوتا ہے اگر subscriber، publisher سے سست ہو؟

## Troubleshooting

**مسئلہ**: Subscriber کو messages موصول نہیں ہو رہے
```bash
# Check QoS compatibility
ros2 topic info /topic_name --verbose
```

**مسئلہ**: Messages بے ترتیبی سے پہنچ رہے ہیں
**حل**: History `KEEP_ALL` کے ساتھ `RELIABLE` QoS استعمال کریں

**مسئلہ**: Memory کا استعمال بڑھ رہا ہے
**حل**: History depth کو `KEEP_LAST(n)` سے محدود کریں

## اگلے قدم

بہترین کام! اب آپ ROS 2 میں pub-sub communication کو سمجھتے ہیں۔

اگلا ہفتہ: [ہفتہ 5: Services، Actions، اور Parameters](week-05.md)

ہم synchronous communication patterns دریافت کریں گے اور ایک مکمل robot control سسٹم بنائیں گے!

## وسائل

- [ROS 2 Topics Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
- [About QoS Settings](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)
- [Common Interfaces (Message Types)](https://github.com/ros2/common_interfaces)
- [Creating Custom Messages](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)

---

## 📝 ہفتہ وار کوئز

اس ہفتے کے مواد کی اپنی سمجھ کو جانچیں! کوئز multiple choice ہے، خودکار طریقے سے اسکور کیا جاتا ہے، اور آپ کے پاس 2 کوششیں ہیں۔

**[ہفتہ 4 کوئز لیں →](/quiz?week=4)**
