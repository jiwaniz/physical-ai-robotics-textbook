# ہفتہ 5: سروسز، ایکشنز، اور پیرامیٹرز

## جائزہ

یہ ہفتہ سینکرونس کمیونیکیشن (سروسز)، طویل چلنے والے کام (ایکشنز)، اور رن ٹائم کنفیگریشن (پیرامیٹرز) کو کور کر کے آپ کے آر او ایس 2 بنیادی اصولوں کو مکمل کرتا ہے۔ آپ ملٹی نوڈ سسٹمز کے انتظام کے لیے لانچ فائلز بھی سیکھیں گے اور باب 1 کا اسیسمنٹ پراجیکٹ مکمل کریں گے۔

## سیکھنے کے مقاصد

اس ہفتے کے اختتام تک، آپ یہ کر سکیں گے:

- ریکویسٹ-رسپانس کمیونیکیشن کے لیے آر او ایس 2 سروسز کو امپلیمنٹ کرنا
- فیڈ بیک کے ساتھ طویل چلنے والے، کینسل ایبل کاموں کے لیے ایکشنز استعمال کرنا
- رن ٹائم کنفیگریشن کے لیے پیرامیٹرز کا انتظام کرنا
- پیچیدہ ملٹی نوڈ سسٹمز شروع کرنے کے لیے لانچ فائلز لکھنا
- مکمل روبوٹک ایپلیکیشن بنانے کے لیے آر او ایس 2 پیٹرنز لاگو کرنا
- باب 1 آر او ایس 2 پراجیکٹ اسیسمنٹ مکمل کرنا

## سروسز: ریکویسٹ-رسپانس کمیونیکیشن

### سروسز بمقابلہ ٹاپکس کب استعمال کریں

| پیٹرن | استعمال کا معاملہ | مثال |
|---------|----------|---------|
| **ٹاپک** | مسلسل ڈیٹا سٹریمز | کیمرا امیجز، لائیڈار اسکینز |
| **سروس** | کبھی کبھار کی کمپیوٹیشنز | پاتھ پلاننگ، آبجیکٹ ریکگنیشن |
| **ایکشن** | فیڈ بیک کے ساتھ طویل کام | نیویگیشن، گراسپنگ |

### سروس ڈیفینیشن

سروسز کے تین اجزاء ہیں:
1. **ریکویسٹ**: کلائنٹ سے سرور کو بھیجا گیا ڈیٹا
2. **رسپانس**: سرور سے کلائنٹ کو واپس کیا گیا ڈیٹا
3. **سروس ٹائپ**: ریکویسٹ اور رسپانس کی ساخت کی تعریف کرتا ہے

**مثال:** `AddTwoInts.srv`
```
# Request
int64 a
int64 b
---
# Response
int64 sum
```

### سروس سرور بنانا

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

### سروس کلائنٹ بنانا

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

### سروسز چلانا

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

### کسٹم سروس ڈیفینیشن

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

## ایکشنز: طویل چلنے والے کام

ایکشنز ایسے کاموں کے لیے سروسز کو بڑھاتے ہیں جو:
- کافی وقت لیتے ہیں (سیکنڈز سے منٹوں تک)
- پروگریس فیڈ بیک فراہم کرتے ہیں
- ایگزیکیوشن کے دوران کینسل کیے جا سکتے ہیں

### ایکشن اسٹرکچر

```
Goal     →  کون سا کام انجام دینا ہے
Feedback →  ایگزیکیوشن کے دوران پروگریس اپڈیٹس
Result   →  مکمل ہونے پر حتمی نتیجہ
```

### مثال: فبوناچی ایکشن

**ڈیفینیشن:** `Fibonacci.action`
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

### ایکشن سرور

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

### ایکشن کلائنٹ

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

### ایکشنز کو کینسل کرنا

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

## پیرامیٹرز: رن ٹائم کنفیگریشن

پیرامیٹرز، نوڈ کے رویے کو ری کمپائلنگ کے بغیر تبدیل کرنے کی اجازت دیتے ہیں۔

### پیرامیٹرز کا اعلان کرنا

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

### پیرامیٹرز سیٹ کرنا

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

### پیرامیٹر کال بیکس

رن ٹائم پر پیرامیٹر تبدیلیوں پر ردعمل ظاہر کریں:

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

## لانچ فائلز

لانچ فائلز متعدد نوڈز کو کنفیگریشنز کے ساتھ شروع کرتی ہیں۔

### پائتھون لانچ فائل

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

### لانچ فائلز چلانا

```bash
# Run launch file
ros2 launch my_robot robot_launch.py

# With arguments
ros2 launch my_robot robot_launch.py robot_name:=Atlas

# List launch files in package
ros2 launch my_robot --show-args
```

### ایکس ایم ایل لانچ فائلز (متبادل)

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

## باب 1 اسیسمنٹ پراجیکٹ

### پراجیکٹ کی ضروریات

**ملٹی نوڈ ڈیلیوری روبوٹ سسٹم** بنائیں جس میں:

**نوڈز (کم از کم 3):**
1. **پیکیج ٹریکر**: پیکیج لوکیشنز اور اسٹیٹس کو ٹریک کرتا ہے
2. **روٹ پلانر**: ڈیلیوری روٹس پلان کرتا ہے (سروس)
3. **ڈیلیوری ایگزیکیوٹر**: ڈیلیوری ٹاسکس ایگزیکیوٹ کرتا ہے (فیڈ بیک کے ساتھ ایکشن)
4. **اسٹیٹس مانیٹر**: سسٹم کی اسٹیٹس لاگ کرتا ہے

**کمیونیکیشن:**
- ٹاپکس: پیکیج اسٹیٹس اپڈیٹس
- سروس: روٹ پلاننگ ریکویسٹ/رسپانس
- ایکشن: پروگریس فیڈ بیک کے ساتھ ڈیلیوری ٹاسک
- پیرامیٹرز: روبوٹ کنفیگریشن (اسپیڈ، کپیسٹی، وغیرہ)

**خصوصیات:**
- کسٹم میسج `PackageInfo` (آئی ڈی، ڈیسٹنیشن، اسٹیٹس)
- کسٹم سروس `PlanRoute` (اسٹارٹ، گول → وے پوائنٹس)
- کسٹم ایکشن `DeliverPackage` (package_id → فیڈ بیک: ڈسٹنس، رزلٹ: سکسیس)
- تمام نوڈز شروع کرنے والی لانچ فائل

**ڈیلیوریبلز:**
1. مکمل آر او ایس 2 پیکیج کے ساتھ گٹ ہب ریپوزٹری
2. سیٹ اپ اور استعمال کی ہدایات کے ساتھ ریڈمی
3. ڈیمو ویڈیو (3-5 منٹ) جو سسٹم کو عمل میں دکھائے
4. آرکیٹیکچر کی وضاحت کرتی تحریری رپورٹ (2-3 صفحات)

**روبرک (100 پوائنٹس):**
- فنکشنیلٹی (40 پوائنٹس): تمام نوڈز صحیح طریقے سے کام کرتے ہیں
- کوڈ کوالٹی (25 پوائنٹس): صاف، دستاویز شدہ، بہترین طریقوں کی پیروی کرتا ہے
- دستاویزات (20 پوائنٹس): واضح ریڈمی اور ان لائن کمنٹس
- ٹیسٹنگ (15 پوائنٹس): اہم اجزاء کے لیے یونٹ ٹیسٹس

**جمع کرانے کی آخری تاریخ**: ہفتہ 5 کا اختتام

## ہفتہ 5 کوئز

1. سروس کے بجائے ایکشن کب استعمال کرنا چاہیے؟
2. پیرامیٹر کو ریڈ-اونلی کیسے بنایا جائے؟
3. سینکرونس اور اسینکرونس سروس کالز میں کیا فرق ہے؟
4. پیچیدہ سسٹمز کے لیے لانچ فائلز کیوں اہم ہیں؟
5. کال بیک میں پیرامیٹر ویلیوز کو کیسے ویلیڈیٹ کیا جائے؟

## عام پیٹرنز

### پیٹرن 1: ٹائم آؤٹ کے ساتھ سروس

```python
future = self.client.call_async(request)
rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

if future.done():
    response = future.result()
else:
    self.get_logger().error('Service call timed out')
```

### پیٹرن 2: پیرامیٹر فائل

**`robot_params.yaml`:**
```yaml
my_node:
  ros__parameters:
    robot_name: "PhysicsBot"
    max_speed: 1.5
    sensor_topics: ["camera", "lidar", "imu"]
    debug_mode: false
```

### پیٹرن 3: لائف سائیکل نوڈز

پروڈکشن سسٹمز کے لیے، مینیجڈ لائف سائیکل نوڈز استعمال کریں:

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

باب 1 مکمل کرنے پر مبارکباد! اب آپ کے پاس مضبوط آر او ایس 2 بنیادیں ہیں۔

**اگلا کیا ہے:**
- باب 1 کا اسیسمنٹ پراجیکٹ مکمل کریں
- باب 2 کے لیے تیاری کریں: [گزیبو اور یونٹی سمیولیشن](../02-simulation/index.md)
- ضرورت کے مطابق آر او ایس 2 تصورات کا جائزہ لیں

## وسائل

- [آر او ایس 2 سروسز ٹیوٹوریل](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
- [آر او ایس 2 ایکشنز ٹیوٹوریل](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
- [آر او ایس 2 پیرامیٹرز ٹیوٹوریل](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
- [آر او ایس 2 لانچ فائلز](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
- [لائف سائیکل نوڈز](https://design.ros2.org/articles/node_lifecycle.html)

---

## 📝 ہفتہ وار کوئز

اس ہفتے کے مواد کی اپنی سمجھ کو جانچیں! کوئز ملٹیپل چوائس ہے، خودکار طریقے سے اسکور کیا جاتا ہے، اور آپ کے پاس 2 کوششیں ہیں۔

**[ہفتہ 5 کوئز لیں →](/quiz?week=5)**
