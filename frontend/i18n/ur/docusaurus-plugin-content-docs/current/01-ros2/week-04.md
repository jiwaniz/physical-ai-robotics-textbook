# Week 4: Nodes, Topics, Publishers & Subscribers

## Overview

This week explores the pub-sub (publish-subscribe) communication pattern in depth. You'll learn how to design topic architectures, work with standard message types, implement Quality of Service (QoS) policies, and build a multi-node robot system with sensors and actuators.

## Learning Objectives

By the end of this week, you will be able to:

- Design topic architectures for multi-node systems
- Use standard ROS 2 message types (`std_msgs`, `sensor_msgs`, `geometry_msgs`)
- Create custom message definitions
- Configure QoS (Quality of Service) policies for reliability
- Implement robust publisher and subscriber nodes
- Build a complete sensor-to-actuator pipeline

## The Publish-Subscribe Pattern

### Why Pub-Sub?

The publish-subscribe pattern **decouples** producers and consumers of data:

**Advantages:**
- **Scalability**: Add subscribers without changing publishers
- **Modularity**: Nodes don't need to know about each other
- **Flexibility**: Mix and match nodes easily
- **Parallel processing**: Multiple subscribers process data independently

**Trade-offs:**
- No delivery guarantees (without proper QoS)
- No request-response (use services for that)
- Potential data overload if subscribers are slow

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

ROS 2 provides common message types in standard packages.

### 1. std_msgs: Basic Types

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

**Common std_msgs types:**
- `String`, `Char`
- `Bool`
- `Int8`, `Int16`, `Int32`, `Int64`
- `UInt8`, `UInt16`, `UInt32`, `UInt64`
- `Float32`, `Float64`
- `Header` (timestamp + frame_id)

### 2. geometry_msgs: Spatial Data

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

**Common geometry_msgs types:**
- `Point`, `Point32` (3D position)
- `Quaternion` (orientation)
- `Pose` (position + orientation)
- `PoseStamped` (Pose + Header)
- `Twist` (linear + angular velocity)
- `Transform` (translation + rotation)

### 3. sensor_msgs: Sensor Data

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

**Common sensor_msgs types:**
- `Image`, `CompressedImage`
- `CameraInfo`
- `LaserScan`, `PointCloud2`
- `Imu` (inertial measurement unit)
- `JointState` (robot joint positions/velocities)
- `NavSatFix` (GPS)

### 4. nav_msgs: Navigation Data

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

**Common nav_msgs types:**
- `Odometry` (robot pose + velocity)
- `Path` (sequence of poses)
- `GridCells` (occupancy grid cells)

## Creating Custom Messages

When standard messages don't fit your needs, create custom ones.

### Step 1: Define Message

Create `msg/RobotStatus.msg`:
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

### Step 2: Update package.xml

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

### Step 3: Update CMakeLists.txt (C++) or setup.py (Python)

**For ament_cmake:**
```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs
)
```

**For ament_python:**
You need a separate interface package (recommended pattern: `my_pkg_interfaces`).

### Step 4: Build and Use

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

QoS policies control how messages are delivered between publishers and subscribers.

### QoS Parameters

| Policy | Options | Description |
|--------|---------|-------------|
| **Reliability** | `RELIABLE`, `BEST_EFFORT` | Guarantee delivery or allow packet loss |
| **Durability** | `VOLATILE`, `TRANSIENT_LOCAL` | Keep messages for late-joining subscribers |
| **History** | `KEEP_LAST(n)`, `KEEP_ALL` | Queue size for messages |
| **Deadline** | Duration | Max time between messages |
| **Lifespan** | Duration | Max time message is valid |
| **Liveliness** | `AUTOMATIC`, `MANUAL` | Detect dead publishers |

### Common QoS Profiles

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

Publishers and subscribers must have **compatible** QoS:

| Publisher | Subscriber | Compatible? |
|-----------|------------|-------------|
| RELIABLE | RELIABLE | ✅ Yes |
| RELIABLE | BEST_EFFORT | ✅ Yes (subscriber gets best effort) |
| BEST_EFFORT | RELIABLE | ❌ No |
| BEST_EFFORT | BEST_EFFORT | ✅ Yes |

**Rule of thumb:**
- Sensors → `BEST_EFFORT` (speed over reliability)
- Control → `RELIABLE` (must not lose commands)
- Status → `RELIABLE` (must know robot state)

## Building a Multi-Node Robot System

Let's build a simple mobile robot with sensor and motor nodes.

### System Architecture

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

### Running the System

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

## Best Practices

### 1. Topic Naming Conventions
```
/robot_name/sensor_type/data_type
/robot1/camera/image_raw
/robot1/lidar/scan
/robot1/motor/cmd_vel
```

### 2. Use Headers for Timestamps
```python
from std_msgs.msg import Header

header = Header()
header.stamp = self.get_clock().now().to_msg()
header.frame_id = 'base_link'
```

### 3. Handle Subscription Lifecycle
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

### 4. Use rclpy.spin_once for Fine-Grained Control
```python
while rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.1)
    # Custom processing here
```

## Week 4 Hands-On Project

**Task**: Build a "Line Follower" simulation

**Requirements:**
1. **Sensor node**: Publishes simulated line position (-1.0 to 1.0, left to right)
2. **Controller node**: Subscribes to sensor, publishes steering commands
3. **Visualizer node**: Logs robot path to file
4. Use custom message `LinePosition` with fields: `header`, `position`, `confidence`
5. Implement QoS policies appropriately
6. Add parameter tuning for PID controller gains

**Bonus**: Create launch file to start all nodes simultaneously (covered next week).

## Quiz Questions

1. What is the difference between `RELIABLE` and `BEST_EFFORT` QoS?
2. When should you use `TRANSIENT_LOCAL` durability?
3. Why are headers important in messages?
4. How do you check if a publisher and subscriber have compatible QoS?
5. What happens if a subscriber is slower than the publisher?

## Troubleshooting

**Issue**: Subscriber not receiving messages
```bash
# Check QoS compatibility
ros2 topic info /topic_name --verbose
```

**Issue**: Messages arriving out of order
**Solution**: Use `RELIABLE` QoS with history `KEEP_ALL`

**Issue**: Memory usage growing
**Solution**: Limit history depth with `KEEP_LAST(n)`

## Next Steps

Excellent work! You now understand pub-sub communication in ROS 2.

Next week: [Week 5: Services, Actions, and Parameters](week-05.md)

We'll explore synchronous communication patterns and build a complete robot control system!

## Resources

- [ROS 2 Topics Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
- [About QoS Settings](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)
- [Common Interfaces (Message Types)](https://github.com/ros2/common_interfaces)
- [Creating Custom Messages](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)

---

## 📝 Weekly Quiz

Test your understanding of this week's content! The quiz is multiple choice, auto-scored, and you have 2 attempts.

**[Take the Week 4 Quiz →](/quiz?week=4)**
