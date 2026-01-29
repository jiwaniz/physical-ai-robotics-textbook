# Week 3: ROS 2 Architecture & Core Concepts

## Overview

Welcome to Chapter 1! This week introduces ROS 2 (Robot Operating System 2), the industry-standard middleware for building modular, distributed robotic systems. You'll learn the fundamental architecture, install ROS 2 Humble, and create your first ROS 2 node.

## Learning Objectives

By the end of this week, you will be able to:

- Explain what ROS 2 is and why it's essential for modern robotics
- Understand the ROS 2 architecture (nodes, topics, DDS)
- Install ROS 2 Humble on Ubuntu 22.04
- Create and run a simple ROS 2 Python node
- Use basic ROS 2 command-line tools (ros2 node, ros2 topic, ros2 run)
- Navigate ROS 2 workspaces and package structure

## What is ROS 2?

**ROS 2 (Robot Operating System 2)** is not an operating system, but a **middleware framework** that provides:

- **Communication infrastructure**: Message passing between components
- **Hardware abstraction**: Unified interfaces for sensors/actuators
- **Package management**: Modular, reusable software components
- **Build system**: Compile and manage complex projects
- **Tooling ecosystem**: Visualization (RViz), simulation (Gazebo), debugging

### ROS 1 vs ROS 2: Why the Upgrade?

| Feature | ROS 1 (2007-2020) | ROS 2 (2017-present) |
|---------|-------------------|----------------------|
| **Communication** | Custom TCPROS/UDPROS | DDS (industry standard) |
| **Real-time support** | Limited | Yes (with real-time OS) |
| **Security** | None | Authentication, encryption |
| **Multi-robot** | Difficult | Native support |
| **Embedded systems** | No | Yes (micro-ROS) |
| **Lifecycle management** | Basic | Managed nodes |
| **QoS (Quality of Service)** | None | Configurable reliability |
| **Platform support** | Linux only | Linux, Windows, macOS |

**Key improvements in ROS 2:**
- Production-ready for commercial robots
- Real-time capable for safety-critical systems
- Better security for networked robots
- More flexible communication patterns

## ROS 2 Architecture

### 1. Nodes: The Building Blocks

A **node** is a process that performs a specific task (e.g., read camera, plan path, control motor). Nodes are the fundamental unit of computation in ROS 2.

**Key characteristics:**
- Modular: Each node does one thing well
- Distributed: Nodes can run on different machines
- Language-agnostic: Write in Python, C++, or Rust
- Lifecycle-managed: Start, pause, stop gracefully

**Example node responsibilities:**
- `camera_driver`: Capture images from camera
- `object_detector`: Detect objects in images
- `motion_planner`: Plan collision-free paths
- `motor_controller`: Send commands to motors

### 2. Topics: Asynchronous Message Passing

**Topics** enable publish-subscribe communication:

```
┌──────────────┐         /camera/image          ┌──────────────┐
│   Camera     │────────────────────────────────▶│   Object     │
│   Driver     │      (Image messages)           │   Detector   │
└──────────────┘                                 └──────────────┘
    Publisher                                       Subscriber
```

**Characteristics:**
- **Many-to-many**: Multiple publishers, multiple subscribers
- **Asynchronous**: No waiting for response
- **Typed**: Messages have defined structure (e.g., `sensor_msgs/Image`)
- **Buffered**: QoS policies control message queue behavior

**Use case**: Sensor data streams (camera, lidar, IMU)

### 3. Services: Synchronous Request-Response

**Services** enable client-server communication:

```
┌──────────────┐      Request: "Plan path      ┌──────────────┐
│   Navigation │      from A to B"             │    Motion    │
│    Node      │──────────────────────────────▶│   Planner    │
│              │◀──────────────────────────────│              │
└──────────────┘      Response: [waypoints]    └──────────────┘
    Client                                          Server
```

**Characteristics:**
- **One-to-one**: Single client, single server
- **Synchronous**: Client waits for response
- **Typed**: Request and response have defined structure

**Use case**: Occasional computations (path planning, object recognition)

### 4. Actions: Long-Running Tasks with Feedback

**Actions** extend services for tasks that take time:

```
┌──────────────┐      Goal: "Navigate to X"    ┌──────────────┐
│     UI       │──────────────────────────────▶│  Navigation  │
│   Node       │◀──────────────────────────────│   Action     │
│              │   Feedback: "50% complete"    │   Server     │
│              │◀──────────────────────────────│              │
└──────────────┘      Result: "Success!"       └──────────────┘
  Action Client                                  Action Server
```

**Characteristics:**
- **Feedback**: Progress updates during execution
- **Cancelable**: Client can cancel goal
- **Preemptable**: New goals can override old ones

**Use case**: Robot motions, grasping, navigation

### 5. Parameters: Runtime Configuration

**Parameters** store configuration values that can be changed without recompiling:

```python
# Declare parameter with default value
self.declare_parameter('max_speed', 1.0)

# Get parameter value
max_speed = self.get_parameter('max_speed').value

# Set parameter from command line
ros2 run my_package my_node --ros-args -p max_speed:=2.5
```

**Use case**: Tuning, calibration, environment-specific settings

### 6. DDS: The Communication Layer

ROS 2 uses **DDS (Data Distribution Service)**, a mature middleware standard:

**Benefits:**
- Industry-proven (aerospace, defense, automotive)
- Automatic discovery (no master node like ROS 1)
- QoS policies (reliability, durability, latency)
- Security (authentication, encryption)

**DDS implementations:**
- Fast DDS (default, Eprosima)
- CycloneDDS (Eclipse)
- Connext DDS (RTI, commercial)

## Installing ROS 2 Humble

ROS 2 Humble Hawksbill is the LTS (Long-Term Support) release supported until May 2027.

### Step 1: Set Locale

```bash
locale  # Check current settings
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Step 2: Add ROS 2 Repository

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

### Step 3: Install ROS 2 Humble

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

**Installation takes ~10 minutes and ~2GB disk space.**

### Step 4: Source ROS 2 Setup

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

### Step 5: Test Installation

```bash
# Terminal 1: Run demo talker
ros2 run demo_nodes_cpp talker

# Terminal 2: Run demo listener
ros2 run demo_nodes_py listener
```

**Expected output:**
```
Terminal 1:
[INFO] [talker]: Publishing: 'Hello World: 1'
[INFO] [talker]: Publishing: 'Hello World: 2'

Terminal 2:
[INFO] [listener]: I heard: [Hello World: 1]
[INFO] [listener]: I heard: [Hello World: 2]
```

If you see messages passing between nodes, ROS 2 is working!

## Creating Your First ROS 2 Node

Let's build a simple "Hello Robot" node.

### Step 1: Create Workspace

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### Step 2: Create Package

```bash
# Create Python package
ros2 pkg create --build-type ament_python hello_robot_py \
  --dependencies rclpy std_msgs

# Navigate into package
cd hello_robot_py
```

**Directory structure:**
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

### Step 3: Create Node Script

```bash
# Create node file
touch hello_robot_py/hello_node.py
chmod +x hello_robot_py/hello_node.py
```

**Edit `hello_robot_py/hello_node.py`:**

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

### Step 4: Update Setup Files

**Edit `setup.py`** - add entry point:

```python
entry_points={
    'console_scripts': [
        'hello_node = hello_robot_py.hello_node:main',
    ],
},
```

### Step 5: Build Package

```bash
# Navigate to workspace root
cd ~/ros2_ws

# Build package
colcon build --packages-select hello_robot_py

# Source workspace overlay
source install/setup.bash
```

### Step 6: Run Your Node

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

**Congratulations! You've created your first ROS 2 node!** 🎉

## Essential ROS 2 Command-Line Tools

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

### General Commands
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

**Best practices:**
- Only commit `src/` to version control
- Add `build/`, `install/`, `log/` to `.gitignore`
- Use separate workspaces for different projects

## Understanding ROS 2 Packages

A **package** is the smallest unit of build and release in ROS 2.

**Package contents:**
- `package.xml`: Metadata (name, version, dependencies)
- `CMakeLists.txt` (C++) or `setup.py` (Python): Build configuration
- Source code: Node implementations
- Launch files: Start multiple nodes
- Config files: Parameters, URDF models

**Package types:**
- **ament_python**: Pure Python packages
- **ament_cmake**: C++ packages or mixed
- **ament_cmake_python**: C++ with Python nodes

## Common Pitfalls & Solutions

### Issue 1: "Package not found" after building
**Cause**: Forgot to source workspace
**Solution**: `source ~/ros2_ws/install/setup.bash`

### Issue 2: Node doesn't receive messages
**Cause**: QoS mismatch between publisher/subscriber
**Solution**: Ensure both use compatible QoS settings (covered next week)

### Issue 3: "colcon: command not found"
**Cause**: ROS 2 dev tools not installed
**Solution**: `sudo apt install python3-colcon-common-extensions`

### Issue 4: ImportError when running Python node
**Cause**: Package not properly installed
**Solution**: Rebuild with `colcon build --symlink-install`

## Week 3 Hands-On Exercise

**Task**: Modify the Hello Robot node to:
1. Accept a parameter `robot_name` (default: "PhysicsBot")
2. Include robot name in status messages
3. Publish at configurable rate (parameter `publish_rate`, default: 1.0 Hz)

**Bonus**: Create a second node that subscribes to `/robot_status` and logs received messages.

**Submission**: Push code to GitHub and share repository link.

## Quiz Questions

1. What is the fundamental unit of computation in ROS 2?
2. Explain the difference between topics and services.
3. Why did ROS 2 adopt DDS instead of custom protocols?
4. What command shows all active topics?
5. What is the purpose of `colcon build`?

## Next Steps

Great job completing Week 3! You now understand ROS 2 architecture and have a working development environment.

Next week: [Week 4: Nodes, Topics, Publishers & Subscribers](week-04.md)

We'll dive deeper into pub-sub communication, message types, and build a multi-node robot system!

## Resources

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [Understanding ROS 2 Nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
- [ROS 2 Design Documents](https://design.ros2.org/)
- [DDS Specification](https://www.omg.org/spec/DDS/)

---

## 📝 Weekly Quiz

Test your understanding of this week's content! The quiz is multiple choice, auto-scored, and you have 2 attempts.

**[Take the Week 3 Quiz →](/quiz?week=3)**
