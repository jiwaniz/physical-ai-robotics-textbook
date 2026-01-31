# ہفتہ 5: Services، Actions، اور Parameters

## جائزہ

یہ ہفتہ synchronous communication (services)، طویل چلنے والے کام (actions)، اور runtime configuration (parameters) کو cover کر کے آپ کے ROS 2 بنیادی اصولوں کو مکمل کرتا ہے۔ آپ multi-node سسٹمز کے انتظام کے لیے launch files بھی سیکھیں گے اور باب 1 کا assessment project مکمل کریں گے۔

## سیکھنے کے مقاصد

اس ہفتے کے اختتام تک، آپ یہ کر سکیں گے:

- Request-response communication کے لیے ROS 2 services کو implement کرنا
- Feedback کے ساتھ طویل چلنے والے، cancelable کاموں کے لیے actions استعمال کرنا
- Runtime configuration کے لیے parameters کا انتظام کرنا
- پیچیدہ multi-node سسٹمز شروع کرنے کے لیے launch files لکھنا
- مکمل robotic application بنانے کے لیے ROS 2 patterns لاگو کرنا
- باب 1 ROS 2 project assessment مکمل کرنا

## Services: Request-Response Communication

### Services بمقابلہ Topics کب استعمال کریں

| Pattern | استعمال کا معاملہ | مثال |
|---------|----------|---------|
| **Topic** | مسلسل ڈیٹا سٹریمز | Camera images، lidar scans |
| **Service** | کبھی کبھار کی computations | Path planning، object recognition |
| **Action** | Feedback کے ساتھ طویل کام | Navigation، grasping |

### Service Definition

Services کے تین اجزاء ہیں:
1. **Request**: Client سے server کو بھیجا گیا ڈیٹا
2. **Response**: Server سے client کو واپس کیا گیا ڈیٹا
3. **Service type**: Request اور response کی ساخت کی تعریف کرتا ہے

**مثال:** `AddTwoInts.srv`
```
# Request
int64 a
int64 b
---
# Response
int64 sum
```

### Service Server بنانا

**`add_two_ints_server.py`:**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServer(Node):
    """
    Service server that adds two integers.
    """

    def __init__(self):
        super().__init__('add_two_ints_server')

        # Create service
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

        self.get_logger().info('Add Two Ints service ready')

    def add_two_ints_callback(self, request, response):
        """
        Service callback: receives request, returns response.
        """
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Incoming request: {request.a} + {request.b} = {response.sum}'
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
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

### Service Client بنانا

**`add_two_ints_client.py`:**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import sys


class AddTwoIntsClient(Node):
    """
    Service client that calls add_two_ints service.
    """

    def __init__(self):
        super().__init__('add_two_ints_client')

        # Create client
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.get_logger().info('Service client ready')

    def send_request(self, a, b):
        """
        Send service request and wait for response.
        """
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        self.get_logger().info(f'Sending request: {a} + {b}')

        # Call service asynchronously
        future = self.client.call_async(request)
        return future


def main(args=None):
    rclpy.init(args=args)

    # Get arguments from command line
    if len(sys.argv) != 3:
        print('Usage: ros2 run pkg client <a> <b>')
        return

    a = int(sys.argv[1])
    b = int(sys.argv[2])

    node = AddTwoIntsClient()

    # Send request
    future = node.send_request(a, b)

    # Wait for response
    rclpy.spin_until_future_complete(node, future)

    try:
        response = future.result()
        node.get_logger().info(f'Result: {a} + {b} = {response.sum}')
    except Exception as e:
        node.get_logger().error(f'Service call failed: {e}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Services چلانا

```bash
# Terminal 1: Start server
ros2 run my_package add_two_ints_server

# Terminal 2: Call from client
ros2 run my_package add_two_ints_client 5 7
# Output: Result: 5 + 7 = 12

# Terminal 3: Call from command line
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 10, b: 20}"
# Output: sum: 30
```

### Custom Service Definition

**`SetRobotMode.srv`:**
```
# Request
string mode  # "manual", "autonomous", "idle"
---
# Response
bool success
string message
```

```python
from my_interfaces.srv import SetRobotMode

def set_mode_callback(self, request, response):
    mode = request.mode
    if mode in ['manual', 'autonomous', 'idle']:
        self.current_mode = mode
        response.success = True
        response.message = f'Mode set to {mode}'
    else:
        response.success = False
        response.message = f'Invalid mode: {mode}'
    return response
```

## Actions: طویل چلنے والے کام

Actions ایسے کاموں کے لیے services کو بڑھاتے ہیں جو:
- کافی وقت لیتے ہیں (سیکنڈز سے منٹوں تک)
- Progress feedback فراہم کرتے ہیں
- Execution کے دوران cancel کیے جا سکتے ہیں

### Action Structure

```
Goal     →  کون سا کام انجام دینا ہے
Feedback →  Execution کے دوران progress updates
Result   →  مکمل ہونے پر حتمی نتیجہ
```

### مثال: Fibonacci Action

**Definition:** `Fibonacci.action`
```
# Goal
int32 order
---
# Result
int32[] sequence
---
# Feedback
int32[] partial_sequence
```

### Action Server

```python
#!/usr/bin/env python3
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci


class FibonacciActionServer(Node):
    """
    Action server that computes Fibonacci sequence.
    """

    def __init__(self):
        super().__init__('fibonacci_action_server')

        # Create action server
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )

        self.get_logger().info('Fibonacci action server ready')

    def execute_callback(self, goal_handle):
        """
        Execute the action goal.
        """
        self.get_logger().info(f'Executing goal: order={goal_handle.request.order}')

        # Initialize feedback
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        # Compute Fibonacci sequence
        for i in range(1, goal_handle.request.order):
            # Check if goal was canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            # Compute next number
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i - 1]
            )

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.partial_sequence}')

            # Simulate processing time
            time.sleep(1.0)

        # Set goal as succeeded
        goal_handle.succeed()

        # Return result
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        self.get_logger().info(f'Goal succeeded! Result: {result.sequence}')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionServer()
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

### Action Client

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci


class FibonacciActionClient(Node):
    """
    Action client that sends Fibonacci goals.
    """

    def __init__(self):
        super().__init__('fibonacci_action_client')

        # Create action client
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci'
        )

    def send_goal(self, order):
        """Send action goal and handle feedback."""
        self.get_logger().info(f'Sending goal: order={order}')

        # Wait for server
        self._action_client.wait_for_server()

        # Create goal
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        # Send goal with callbacks
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Called when server accepts/rejects goal."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Called when server publishes feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.partial_sequence}')

    def get_result_callback(self, future):
        """Called when action completes."""
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionClient()
    node.send_goal(10)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
```

### Actions کو Cancel کرنا

```python
# In action client
def cancel_goal(self, goal_handle):
    """Cancel active goal."""
    cancel_future = goal_handle.cancel_goal_async()
    cancel_future.add_done_callback(self.cancel_done)

def cancel_done(self, future):
    cancel_response = future.result()
    if cancel_response.goals_canceling:
        self.get_logger().info('Goal successfully canceled')
```

## Parameters: Runtime Configuration

Parameters، node کے رویے کو recompiling کے بغیر تبدیل کرنے کی اجازت دیتے ہیں۔

### Parameters کا اعلان کرنا

```python
def __init__(self):
    super().__init__('my_node')

    # Declare parameters with defaults
    self.declare_parameter('robot_name', 'PhysicsBot')
    self.declare_parameter('max_speed', 1.0)
    self.declare_parameter('debug_mode', False)
    self.declare_parameter('sensor_topics', ['camera', 'lidar'])

    # Get parameter values
    self.robot_name = self.get_parameter('robot_name').value
    self.max_speed = self.get_parameter('max_speed').value
    self.debug_mode = self.get_parameter('debug_mode').value
    self.sensor_topics = self.get_parameter('sensor_topics').value

    self.get_logger().info(f'Robot: {self.robot_name}, Max speed: {self.max_speed}')
```

### Parameters سیٹ کرنا

```bash
# Command line (when launching node)
ros2 run pkg node --ros-args -p robot_name:=Atlas -p max_speed:=2.5

# Runtime (after node is running)
ros2 param set /my_node max_speed 3.0

# List parameters
ros2 param list /my_node

# Get parameter value
ros2 param get /my_node max_speed

# Dump all parameters to file
ros2 param dump /my_node > my_params.yaml

# Load parameters from file
ros2 run pkg node --ros-args --params-file my_params.yaml
```

### Parameter Callbacks

Runtime پر parameter تبدیلیوں پر ردعمل ظاہر کریں:

```python
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

def __init__(self):
    super().__init__('my_node')

    # Declare parameter with descriptor
    descriptor = ParameterDescriptor(
        description='Maximum robot speed in m/s',
        type=ParameterType.PARAMETER_DOUBLE
    )
    self.declare_parameter('max_speed', 1.0, descriptor)

    # Add callback for parameter changes
    self.add_on_set_parameters_callback(self.parameter_callback)

def parameter_callback(self, params):
    """Called when parameters are modified."""
    for param in params:
        if param.name == 'max_speed':
            if param.value < 0.0 or param.value > 5.0:
                return SetParametersResult(successful=False)
            self.max_speed = param.value
            self.get_logger().info(f'Max speed updated to {self.max_speed}')

    return SetParametersResult(successful=True)
```

## Launch Files

Launch files متعدد nodes کو configurations کے ساتھ شروع کرتی ہیں۔

### Python Launch File

**`robot_launch.py`:**

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch robot system with multiple nodes.
    """

    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='PhysicsBot',
        description='Name of the robot'
    )

    # Get launch configurations
    robot_name = LaunchConfiguration('robot_name')

    # Node 1: Distance sensor
    sensor_node = Node(
        package='my_robot',
        executable='distance_sensor_node',
        name='distance_sensor',
        parameters=[{
            'publish_rate': 10.0,
            'min_range': 0.1,
            'max_range': 5.0
        }],
        output='screen'
    )

    # Node 2: Obstacle detector
    detector_node = Node(
        package='my_robot',
        executable='obstacle_detector_node',
        name='obstacle_detector',
        parameters=[{
            'safety_distance': 0.5,
            'max_speed': 0.5
        }],
        output='screen'
    )

    # Node 3: Motor controller
    motor_node = Node(
        package='my_robot',
        executable='motor_controller_node',
        name='motor_controller',
        parameters=[{
            'robot_name': robot_name
        }],
        output='screen'
    )

    # Node 4: RViz for visualization (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '/path/to/config.rviz'],
        output='screen'
    )

    return LaunchDescription([
        robot_name_arg,
        sensor_node,
        detector_node,
        motor_node,
        # rviz_node,  # Uncomment to enable
    ])
```

### Launch Files چلانا

```bash
# Run launch file
ros2 launch my_robot robot_launch.py

# With arguments
ros2 launch my_robot robot_launch.py robot_name:=Atlas

# List launch files in package
ros2 launch my_robot --show-args
```

### XML Launch Files (متبادل)

```xml
<launch>
  <arg name="robot_name" default="PhysicsBot"/>

  <node pkg="my_robot" exec="distance_sensor_node" name="distance_sensor">
    <param name="publish_rate" value="10.0"/>
  </node>

  <node pkg="my_robot" exec="obstacle_detector_node" name="obstacle_detector">
    <param name="safety_distance" value="0.5"/>
  </node>

  <node pkg="my_robot" exec="motor_controller_node" name="motor_controller">
    <param name="robot_name" value="$(var robot_name)"/>
  </node>
</launch>
```

## باب 1 Assessment Project

### پراجیکٹ کی ضروریات

**Multi-node delivery robot سسٹم** بنائیں جس میں:

**Nodes (کم از کم 3):**
1. **Package tracker**: Package locations اور status کو track کرتا ہے
2. **Route planner**: Delivery routes plan کرتا ہے (service)
3. **Delivery executor**: Delivery tasks execute کرتا ہے (feedback کے ساتھ action)
4. **Status monitor**: سسٹم کی status log کرتا ہے

**Communication:**
- Topics: Package status updates
- Service: Route planning request/response
- Action: Progress feedback کے ساتھ delivery task
- Parameters: Robot configuration (speed، capacity، وغیرہ)

**خصوصیات:**
- Custom message `PackageInfo` (id، destination، status)
- Custom service `PlanRoute` (start، goal → waypoints)
- Custom action `DeliverPackage` (package_id → feedback: distance، result: success)
- تمام nodes شروع کرنے والی launch file

**Deliverables:**
1. مکمل ROS 2 package کے ساتھ GitHub repository
2. Setup اور استعمال کی ہدایات کے ساتھ README
3. Demo video (3-5 منٹ) جو سسٹم کو عمل میں دکھائے
4. آرکیٹیکچر کی وضاحت کرتی تحریری رپورٹ (2-3 صفحات)

**Rubric (100 پوائنٹس):**
- Functionality (40 pts): تمام nodes صحیح طریقے سے کام کرتے ہیں
- Code quality (25 pts): صاف، دستاویز شدہ، بہترین طریقوں کی پیروی کرتا ہے
- Documentation (20 pts): واضح README اور inline comments
- Testing (15 pts): اہم اجزاء کے لیے unit tests

**جمع کرانے کی آخری تاریخ**: ہفتہ 5 کا اختتام

## ہفتہ 5 کوئز

1. Service کے بجائے action کب استعمال کرنا چاہیے؟
2. Parameter کو read-only کیسے بنایا جائے؟
3. Synchronous اور asynchronous service calls میں کیا فرق ہے؟
4. پیچیدہ سسٹمز کے لیے launch files کیوں اہم ہیں؟
5. Callback میں parameter values کو کیسے validate کیا جائے؟

## عام Patterns

### Pattern 1: Timeout کے ساتھ Service

```python
future = self.client.call_async(request)
rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

if future.done():
    response = future.result()
else:
    self.get_logger().error('Service call timed out')
```

### Pattern 2: Parameter File

**`robot_params.yaml`:**
```yaml
my_node:
  ros__parameters:
    robot_name: "PhysicsBot"
    max_speed: 1.5
    sensor_topics: ["camera", "lidar", "imu"]
    debug_mode: false
```

### Pattern 3: Lifecycle Nodes

Production سسٹمز کے لیے، managed lifecycle nodes استعمال کریں:

```python
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn

class MyLifecycleNode(LifecycleNode):
    def on_configure(self, state: LifecycleState):
        self.get_logger().info('Configuring...')
        # Initialize resources
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState):
        self.get_logger().info('Activating...')
        # Start publishers, timers
        return TransitionCallbackReturn.SUCCESS
```

## اگلے قدم

باب 1 مکمل کرنے پر مبارکباد! اب آپ کے پاس مضبوط ROS 2 بنیادیں ہیں۔

**اگلا کیا ہے:**
- باب 1 کا assessment project مکمل کریں
- باب 2 کے لیے تیاری کریں: [Gazebo اور Unity Simulation](../02-simulation/index.md)
- ضرورت کے مطابق ROS 2 تصورات کا جائزہ لیں

## وسائل

- [ROS 2 Services Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
- [ROS 2 Actions Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
- [ROS 2 Parameters Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
- [ROS 2 Launch Files](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
- [Lifecycle Nodes](https://design.ros2.org/articles/node_lifecycle.html)

---

## 📝 ہفتہ وار کوئز

اس ہفتے کے مواد کی اپنی سمجھ کو جانچیں! کوئز multiple choice ہے، خودکار طریقے سے اسکور کیا جاتا ہے، اور آپ کے پاس 2 کوششیں ہیں۔

**[ہفتہ 5 کوئز لیں →](/quiz?week=5)**
