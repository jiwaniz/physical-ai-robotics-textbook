# ہفتہ 3: ROS 2 آرکیٹیکچر اور بنیادی تصورات

## جائزہ

باب 1 میں خوش آمدید! اس ہفتے ROS 2 (Robot Operating System 2) متعارف کرایا جاتا ہے، جو modular، distributed robotic سسٹمز بنانے کے لیے industry-standard middleware ہے۔ آپ بنیادی آرکیٹیکچر سیکھیں گے، ROS 2 Humble install کریں گے، اور اپنا پہلا ROS 2 node بنائیں گے۔

## سیکھنے کے مقاصد

اس ہفتے کے اختتام تک، آپ یہ کر سکیں گے:

- ROS 2 کیا ہے اور جدید روبوٹکس کے لیے یہ کیوں ضروری ہے، وضاحت کرنا
- ROS 2 آرکیٹیکچر (nodes، topics، DDS) کو سمجھنا
- Ubuntu 22.04 پر ROS 2 Humble install کرنا
- ایک سادہ ROS 2 Python node بنانا اور چلانا
- بنیادی ROS 2 command-line tools استعمال کرنا (ros2 node، ros2 topic، ros2 run)
- ROS 2 workspaces اور package structure میں navigate کرنا

## ROS 2 کیا ہے؟

**ROS 2 (Robot Operating System 2)** کوئی operating system نہیں، بلکہ ایک **middleware framework** ہے جو فراہم کرتا ہے:

- **Communication infrastructure**: اجزاء کے درمیان message passing
- **Hardware abstraction**: سینسرز/ایکچویٹرز کے لیے یکساں interfaces
- **Package management**: Modular، دوبارہ استعمال کے قابل software components
- **Build system**: پیچیدہ projects کو compile اور manage کرنا
- **Tooling ecosystem**: Visualization (RViz)، simulation (Gazebo)، debugging

### ROS 1 بمقابلہ ROS 2: اپ گریڈ کیوں؟

| Feature | ROS 1 (2007-2020) | ROS 2 (2017-موجودہ) |
|---------|-------------------|----------------------|
| **Communication** | Custom TCPROS/UDPROS | DDS (industry standard) |
| **Real-time support** | محدود | ہاں (real-time OS کے ساتھ) |
| **Security** | کوئی نہیں | Authentication، encryption |
| **Multi-robot** | مشکل | Native support |
| **Embedded systems** | نہیں | ہاں (micro-ROS) |
| **Lifecycle management** | بنیادی | Managed nodes |
| **QoS (Quality of Service)** | کوئی نہیں | Configurable reliability |
| **Platform support** | صرف Linux | Linux، Windows، macOS |

**ROS 2 میں اہم بہتریاں:**
- تجارتی روبوٹس کے لیے production-ready
- حفاظتی لحاظ سے اہم نظاموں کے لیے real-time capable
- نیٹ ورک شدہ روبوٹس کے لیے بہتر سیکیورٹی
- زیادہ لچکدار communication patterns

## ROS 2 آرکیٹیکچر

### 1. Nodes: بلڈنگ بلاکس

ایک **node** ایک process ہے جو ایک مخصوص کام انجام دیتا ہے (مثلاً، camera پڑھنا، path plan کرنا، motor control کرنا)۔ Nodes، ROS 2 میں computation کی بنیادی اکائی ہیں۔

**اہم خصوصیات:**
- Modular: ہر node ایک کام اچھی طرح کرتا ہے
- Distributed: Nodes مختلف machines پر چل سکتے ہیں
- Language-agnostic: Python، C++، یا Rust میں لکھیں
- Lifecycle-managed: شائستگی سے start، pause، stop

**Node ذمہ داریوں کی مثال:**
- `camera_driver`: Camera سے images capture کرنا
- `object_detector`: Images میں objects detect کرنا
- `motion_planner`: Collision-free paths plan کرنا
- `motor_controller`: Motors کو commands بھیجنا

### 2. Topics: Asynchronous Message Passing

**Topics** publish-subscribe communication کو قابل بناتے ہیں:

```
┌──────────────┐         /camera/image          ┌──────────────┐
│   Camera     │────────────────────────────────▶│   Object     │
│   Driver     │      (Image messages)           │   Detector   │
└──────────────┘                                 └──────────────┘
    Publisher                                       Subscriber
```

**خصوصیات:**
- **Many-to-many**: متعدد publishers، متعدد subscribers
- **Asynchronous**: جواب کا انتظار نہیں
- **Typed**: Messages کی متعین ساخت ہے (مثلاً، `sensor_msgs/Image`)
- **Buffered**: QoS policies message queue کے رویے کو کنٹرول کرتی ہیں

**استعمال کا معاملہ**: سینسر ڈیٹا سٹریمز (camera، lidar، IMU)

### 3. Services: Synchronous Request-Response

**Services** client-server communication کو قابل بناتی ہیں:

```
┌──────────────┐      Request: "Plan path      ┌──────────────┐
│   Navigation │      from A to B"             │    Motion    │
│    Node      │──────────────────────────────▶│   Planner    │
│              │◀──────────────────────────────│              │
└──────────────┘      Response: [waypoints]    └──────────────┘
    Client                                          Server
```

**خصوصیات:**
- **One-to-one**: ایک client، ایک server
- **Synchronous**: Client جواب کا انتظار کرتا ہے
- **Typed**: Request اور response کی متعین ساخت ہے

**استعمال کا معاملہ**: کبھی کبھار کی computations (path planning، object recognition)

### 4. Actions: طویل چلنے والے کام Feedback کے ساتھ

**Actions** ایسے کاموں کے لیے services کو بڑھاتے ہیں جو وقت لیتے ہیں:

```
┌──────────────┐      Goal: "Navigate to X"    ┌──────────────┐
│     UI       │──────────────────────────────▶│  Navigation  │
│   Node       │◀──────────────────────────────│   Action     │
│              │   Feedback: "50% complete"    │   Server     │
│              │◀──────────────────────────────│              │
└──────────────┘      Result: "Success!"       └──────────────┘
  Action Client                                  Action Server
```

**خصوصیات:**
- **Feedback**: Execution کے دوران progress updates
- **Cancelable**: Client goal کو cancel کر سکتا ہے
- **Preemptable**: نئے goals پرانے کو override کر سکتے ہیں

**استعمال کا معاملہ**: روبوٹ motions، grasping، navigation

### 5. Parameters: Runtime Configuration

**Parameters** configuration values کو store کرتے ہیں جو recompiling کے بغیر تبدیل کیے جا سکتے ہیں:

```python
# Declare parameter with default value
self.declare_parameter('max_speed', 1.0)

# Get parameter value
max_speed = self.get_parameter('max_speed').value

# Set parameter from command line
ros2 run my_package my_node --ros-args -p max_speed:=2.5
```

**استعمال کا معاملہ**: Tuning، calibration، environment-specific settings

### 6. DDS: Communication Layer

ROS 2 **DDS (Data Distribution Service)** استعمال کرتا ہے، ایک پختہ middleware standard:

**فوائد:**
- Industry-proven (aerospace، defense، automotive)
- خودکار discovery (ROS 1 جیسا master node نہیں)
- QoS policies (reliability، durability، latency)
- Security (authentication، encryption)

**DDS implementations:**
- Fast DDS (default، Eprosima)
- CycloneDDS (Eclipse)
- Connext DDS (RTI، commercial)

## ROS 2 Humble انسٹال کرنا

ROS 2 Humble Hawksbill ایک LTS (Long-Term Support) release ہے جو مئی 2027 تک supported ہے۔

### قدم 1: Locale سیٹ کریں

```bash
locale  # Check current settings
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### قدم 2: ROS 2 Repository شامل کریں

```bash
# Ensure Ubuntu Universe repository is enabled
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### قدم 3: ROS 2 Humble انسٹال کریں

```bash
# Update package index
sudo apt update

# Upgrade packages to avoid conflicts
sudo apt upgrade -y

# Install ROS 2 Humble Desktop (includes RViz, demos, tutorials)
sudo apt install ros-humble-desktop -y

# Install development tools
sudo apt install ros-dev-tools -y

# Install colcon (ROS 2 build tool)
sudo apt install python3-colcon-common-extensions -y
```

**تنصیب میں تقریباً 10 منٹ اور 2GB ڈسک جگہ لگتی ہے۔**

### قدم 4: ROS 2 Setup کو Source کریں

```bash
# Source ROS 2 environment (run in every new terminal)
source /opt/ros/humble/setup.bash

# Add to ~/.bashrc for automatic sourcing
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
ros2 --version
# Expected output: ros2 cli version: 0.x.x
```

### قدم 5: تنصیب کا ٹیسٹ کریں

```bash
# Terminal 1: Run demo talker
ros2 run demo_nodes_cpp talker

# Terminal 2: Run demo listener
ros2 run demo_nodes_py listener
```

**متوقع output:**
```
Terminal 1:
[INFO] [talker]: Publishing: 'Hello World: 1'
[INFO] [talker]: Publishing: 'Hello World: 2'

Terminal 2:
[INFO] [listener]: I heard: [Hello World: 1]
[INFO] [listener]: I heard: [Hello World: 2]
```

اگر آپ nodes کے درمیان messages دیکھتے ہیں، تو ROS 2 کام کر رہا ہے!

## اپنا پہلا ROS 2 Node بنانا

آئیے ایک سادہ "Hello Robot" node بنائیں۔

### قدم 1: Workspace بنائیں

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### قدم 2: Package بنائیں

```bash
# Create Python package
ros2 pkg create --build-type ament_python hello_robot_py \
  --dependencies rclpy std_msgs

# Navigate into package
cd hello_robot_py
```

**ڈائریکٹری کی ساخت:**
```
hello_robot_py/
├── package.xml          # Package metadata
├── setup.py             # Python build configuration
├── setup.cfg            # Additional setup config
├── resource/            # Package marker file
├── test/                # Unit tests
└── hello_robot_py/      # Python source code
    └── __init__.py
```

### قدم 3: Node Script بنائیں

```bash
# Create node file
touch hello_robot_py/hello_node.py
chmod +x hello_robot_py/hello_node.py
```

**`hello_robot_py/hello_node.py` میں ترمیم کریں:**

```python
#!/usr/bin/env python3
"""
Simple ROS 2 Node - Hello Robot
Publishes robot status messages every second.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloRobotNode(Node):
    """
    A simple ROS 2 node that publishes robot status messages.
    """

    def __init__(self):
        # Initialize node with name 'hello_robot'
        super().__init__('hello_robot')

        # Create publisher on topic '/robot_status'
        # Queue size: 10 messages
        self.publisher_ = self.create_publisher(String, '/robot_status', 10)

        # Create timer that calls timer_callback every 1.0 seconds
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter for messages
        self.counter = 0

        # Log that node has started
        self.get_logger().info('Hello Robot Node has started!')

    def timer_callback(self):
        """
        Called every timer period. Publishes robot status message.
        """
        # Create message
        msg = String()
        msg.data = f'Robot status update #{self.counter}: All systems operational'

        # Publish message
        self.publisher_.publish(msg)

        # Log to console
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment counter
        self.counter += 1


def main(args=None):
    """
    Main function: Initialize ROS 2, create node, spin.
    """
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create node instance
    node = HelloRobotNode()

    try:
        # Spin node (process callbacks until shutdown)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### قدم 4: Setup Files کو اپ ڈیٹ کریں

**`setup.py` میں ترمیم کریں** - entry point شامل کریں:

```python
entry_points={
    'console_scripts': [
        'hello_node = hello_robot_py.hello_node:main',
    ],
},
```

### قدم 5: Package کو Build کریں

```bash
# Navigate to workspace root
cd ~/ros2_ws

# Build package
colcon build --packages-select hello_robot_py

# Source workspace overlay
source install/setup.bash
```

### قدم 6: اپنا Node چلائیں

```bash
# Terminal 1: Run node
ros2 run hello_robot_py hello_node

# Terminal 2: List active nodes
ros2 node list
# Output: /hello_robot

# Terminal 2: See node info
ros2 node info /hello_robot

# Terminal 2: Echo messages
ros2 topic echo /robot_status
```

**مبارک ہو! آپ نے اپنا پہلا ROS 2 node بنا لیا!** 🎉

## ضروری ROS 2 Command-Line Tools

### Node Commands
```bash
ros2 node list                    # List running nodes
ros2 node info /node_name         # Show node details
```

### Topic Commands
```bash
ros2 topic list                   # List active topics
ros2 topic echo /topic_name       # Print messages
ros2 topic hz /topic_name         # Show publishing rate
ros2 topic info /topic_name       # Show publishers/subscribers
ros2 topic pub /topic_name ...    # Publish message from CLI
```

### Parameter Commands
```bash
ros2 param list                   # List parameters
ros2 param get /node_name param   # Get parameter value
ros2 param set /node_name param value  # Set parameter
```

### عمومی Commands
```bash
ros2 pkg list                     # List installed packages
ros2 interface show Type          # Show message definition
ros2 doctor                       # Check ROS 2 setup
```

## ROS 2 Workspace Structure

```
ros2_ws/                      # Workspace root
├── src/                      # Source code
│   └── hello_robot_py/       # Your package
├── build/                    # Build artifacts (auto-generated)
├── install/                  # Installed packages (auto-generated)
└── log/                      # Build logs (auto-generated)
```

**بہترین طریقے:**
- صرف `src/` کو version control میں commit کریں
- `build/`، `install/`، `log/` کو `.gitignore` میں شامل کریں
- مختلف projects کے لیے الگ workspaces استعمال کریں

## ROS 2 Packages کو سمجھنا

ایک **package** ROS 2 میں build اور release کی سب سے چھوٹی اکائی ہے۔

**Package کے اجزاء:**
- `package.xml`: Metadata (نام، ورژن، dependencies)
- `CMakeLists.txt` (C++) یا `setup.py` (Python): Build configuration
- Source code: Node implementations
- Launch files: متعدد nodes شروع کرنا
- Config files: Parameters، URDF models

**Package کی اقسام:**
- **ament_python**: خالص Python packages
- **ament_cmake**: C++ packages یا mixed
- **ament_cmake_python**: Python nodes کے ساتھ C++

## عام مسائل اور حل

### مسئلہ 1: Building کے بعد "Package not found"
**وجہ**: Workspace کو source کرنا بھول گئے
**حل**: `source ~/ros2_ws/install/setup.bash`

### مسئلہ 2: Node کو messages موصول نہیں ہوتے
**وجہ**: Publisher/subscriber کے درمیان QoS mismatch
**حل**: یقینی بنائیں کہ دونوں compatible QoS settings استعمال کرتے ہیں (اگلے ہفتے covered)

### مسئلہ 3: "colcon: command not found"
**وجہ**: ROS 2 dev tools انسٹال نہیں
**حل**: `sudo apt install python3-colcon-common-extensions`

### مسئلہ 4: Python node چلاتے وقت ImportError
**وجہ**: Package صحیح طریقے سے install نہیں
**حل**: `colcon build --symlink-install` کے ساتھ دوبارہ build کریں

## ہفتہ 3 کی عملی مشق

**کام**: Hello Robot node میں ترمیم کریں تاکہ:
1. ایک parameter `robot_name` قبول کرے (default: "PhysicsBot")
2. Robot کا نام status messages میں شامل کرے
3. قابل تشکیل rate پر publish کرے (parameter `publish_rate`، default: 1.0 Hz)

**بونس**: ایک دوسرا node بنائیں جو `/robot_status` کو subscribe کرے اور موصول شدہ messages کو log کرے۔

**جمع کرانا**: کوڈ کو GitHub پر push کریں اور repository link شیئر کریں۔

## کوئز کے سوالات

1. ROS 2 میں computation کی بنیادی اکائی کیا ہے؟
2. Topics اور services کے درمیان فرق کی وضاحت کریں۔
3. ROS 2 نے custom protocols کے بجائے DDS کیوں اپنایا؟
4. کون سا command تمام فعال topics دکھاتا ہے؟
5. `colcon build` کا مقصد کیا ہے؟

## اگلے قدم

ہفتہ 3 مکمل کرنے پر بہترین کام! اب آپ ROS 2 آرکیٹیکچر کو سمجھتے ہیں اور آپ کے پاس ایک کام کرنے والا development environment ہے۔

اگلا ہفتہ: [ہفتہ 4: Nodes، Topics، Publishers اور Subscribers](week-04.md)

ہم pub-sub communication میں گہرائی سے جائیں گے، message types، اور ایک multi-node robot سسٹم بنائیں گے!

## وسائل

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [Understanding ROS 2 Nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
- [ROS 2 Design Documents](https://design.ros2.org/)
- [DDS Specification](https://www.omg.org/spec/DDS/)

---

## 📝 ہفتہ وار کوئز

اس ہفتے کے مواد کی اپنی سمجھ کو جانچیں! کوئز multiple choice ہے، خودکار طریقے سے اسکور کیا جاتا ہے، اور آپ کے پاس 2 کوششیں ہیں۔

**[ہفتہ 3 کوئز لیں →](/quiz?week=3)**
