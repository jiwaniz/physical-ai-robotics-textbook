# Week 8: NVIDIA Isaac Sim - Introduction & Setup

## Overview

Welcome to Chapter 3! This week introduces NVIDIA Isaac Sim, a GPU-accelerated robotics simulator built on NVIDIA Omniverse. You'll learn why Isaac Sim is revolutionary for Physical AI, install the platform, create your first simulation, and integrate with ROS 2.

## Learning Objectives

By the end of this week, you will be able to:

- Understand what makes Isaac Sim different from Gazebo
- Install NVIDIA Isaac Sim and Omniverse
- Navigate the Isaac Sim interface
- Import and manipulate 3D assets (USD format)
- Create basic robot simulations with physics
- Integrate Isaac Sim with ROS 2
- Use Isaac Sim Python API for automation

## What is NVIDIA Isaac Sim?

**Isaac Sim** is a robotics simulation platform built on NVIDIA Omniverse, leveraging:

- **PhysX 5**: GPU-accelerated physics engine (1000x faster than CPU)
- **RTX Ray Tracing**: Photorealistic rendering for vision AI
- **USD (Universal Scene Description)**: Industry-standard 3D format (Pixar)
- **Python API**: Full programmatic control
- **ROS 2 Integration**: Native bridges for ROS topics/services
- **Synthetic Data Generation**: Automated dataset creation for ML

### Isaac Sim vs Gazebo Classic

| Feature | Gazebo Classic | Isaac Sim |
|---------|---------------|-----------|
| **Physics Engine** | ODE/Bullet (CPU) | PhysX 5 (GPU) |
| **Rendering** | OGRE (basic) | RTX ray tracing (photorealistic) |
| **Parallel Simulations** | Limited | 1000s on GPU |
| **AI/ML Integration** | External | Native (Isaac Gym, Replicator) |
| **Sensor Simulation** | Simplified | Physically accurate (cameras, lidar) |
| **Scene Format** | SDF/URDF | USD (Universal Scene Description) |
| **Extensibility** | C++ plugins | Python API |
| **License** | Open source | Free for research/education |

**When to use Isaac Sim:**
- Training ML models with synthetic data
- Photorealistic vision datasets
- Massively parallel RL (Isaac Gym)
- High-fidelity physics (deformables, fluids)
- Industrial digital twins

**When to use Gazebo:**
- Quick prototyping without GPU
- Legacy ROS 1 workflows
- Open-source requirements
- Lighter-weight simulations

## System Requirements

### Minimum Requirements
- **GPU**: NVIDIA RTX 2060 or higher (6GB VRAM)
- **CPU**: Intel i7 or AMD Ryzen 7
- **RAM**: 32GB
- **Storage**: 50GB SSD free space
- **OS**: Ubuntu 20.04/22.04 or Windows 10/11

### Recommended Requirements
- **GPU**: NVIDIA RTX 3080 or higher (12GB+ VRAM)
- **CPU**: Intel i9 or AMD Ryzen 9
- **RAM**: 64GB
- **Storage**: 100GB NVMe SSD

### Cloud Options (if no local GPU)
- **AWS**: g5.xlarge (A10G GPU, $1.006/hr)
- **GCP**: n1-standard-4 + T4 GPU ($0.70/hr)
- **NVIDIA Omniverse Cloud**: Streaming option (pricing varies)

## Installing NVIDIA Isaac Sim

### Step 1: NVIDIA Driver and CUDA

```bash
# Check current driver
nvidia-smi

# Install driver (535+ recommended)
sudo apt install nvidia-driver-535 -y
sudo reboot

# Verify driver
nvidia-smi
# Should show CUDA Version: 12.2 or higher

# Install CUDA Toolkit (optional, for development)
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
sudo apt install cuda-toolkit-12-2 -y
```

### Step 2: Create NVIDIA Account

1. Visit [https://developer.nvidia.com/isaac-sim](https://developer.nvidia.com/isaac-sim)
2. Click "Get Started" → Sign in/Create account
3. Join NVIDIA Developer Program (free)

### Step 3: Install Omniverse Launcher

**Linux:**
```bash
# Download launcher
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# Make executable
chmod +x omniverse-launcher-linux.AppImage

# Run launcher
./omniverse-launcher-linux.AppImage
```

**Windows:**
Download installer from [https://www.nvidia.com/en-us/omniverse/download/](https://www.nvidia.com/en-us/omniverse/download/)

### Step 4: Install Isaac Sim via Launcher

1. Open Omniverse Launcher
2. Go to **Exchange** tab
3. Search "Isaac Sim"
4. Click **Install** (choose version 2023.1.1 or latest)
5. Installation takes ~20-30 minutes (20GB download)

### Step 5: Launch Isaac Sim

1. In Launcher, go to **Library** tab
2. Find "Isaac Sim"
3. Click **Launch**
4. First launch takes 5-10 minutes (shader compilation)

**Verification:**
- Isaac Sim window should open
- You should see the welcome screen
- No error messages in console

### Step 6: Install ROS 2 Bridge

```bash
# Navigate to Isaac Sim directory
cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1

# Run ROS 2 install script
./setup_python_env.sh

# Install ROS 2 Humble bridge
./install_ros2_humble.sh
```

## Isaac Sim Interface Overview

### Main Components

1. **Viewport**: 3D scene visualization
2. **Stage**: Hierarchy of objects (USD prims)
3. **Property Panel**: Object properties and settings
4. **Content Browser**: Asset library
5. **Console**: Python scripts and logs

### Navigation Controls

| Action | Control |
|--------|---------|
| **Orbit camera** | Middle mouse drag |
| **Pan camera** | Shift + Middle mouse drag |
| **Zoom** | Mouse wheel |
| **Select object** | Left click |
| **Multi-select** | Ctrl + Left click |
| **Focus on object** | F key |
| **Frame all** | A key |

### Viewport Modes

- **Lit**: Realistic rendering with lights
- **Wireframe**: Show polygon edges
- **Physics Debug**: Visualize collision shapes
- **Bounds**: Show bounding boxes

## USD: Universal Scene Description

Isaac Sim uses **USD (Universal Scene Description)**, Pixar's open-source 3D format.

### USD Concepts

**Prims (Primitives)**: Everything in the scene is a prim
- `Xform`: Transform node (position, rotation, scale)
- `Mesh`: 3D geometry
- `Material`: Appearance properties
- `Light`: Light sources
- `Camera`: Viewpoints

**Stage**: Container for all prims (the scene)

**Layers**: Non-destructive edits (like Photoshop layers)

### Creating Objects via GUI

1. **Create → Mesh → Cube**
   - Creates cube at origin
   - Visible in Stage panel as `/World/Cube`

2. **Property Panel → Transform**
   - Position: X, Y, Z coordinates
   - Rotation: Euler angles or quaternion
   - Scale: X, Y, Z scaling

3. **Property Panel → Physics**
   - Add **Rigid Body** component
   - Set mass, collision shape, friction

### Example: Build a Simple Scene

**Task:** Create a table with objects

```
1. Create → Mesh → Cube (table surface)
   - Scale: (2, 1, 0.1)
   - Position: (0, 0, 1)

2. Create → Mesh → Cylinder (table leg)
   - Scale: (0.1, 0.1, 1)
   - Position: (0.8, 0.4, 0)
   - Duplicate 3 more times for 4 legs

3. Create → Mesh → Sphere (ball on table)
   - Radius: 0.1
   - Position: (0, 0, 1.15)
   - Add Physics → Rigid Body

4. Create → Physics → Ground Plane

5. Click Play (bottom toolbar) to run physics
```

## Python Scripting in Isaac Sim

Isaac Sim has a built-in Python interpreter with access to the full API.

### Running Python Scripts

**Method 1: Script Editor (GUI)**
1. Window → Script Editor
2. Write Python code
3. Click **Run**

**Method 2: Standalone Python Script**
```bash
# From Isaac Sim directory
./python.sh /path/to/script.py
```

### Hello World Script

```python
# hello_isaac.py
from omni.isaac.kit import SimulationApp

# Launch Isaac Sim headless (no GUI)
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
import numpy as np

# Create world
world = World(stage_units_in_meters=1.0)

# Add ground plane
world.scene.add_default_ground_plane()

# Add a cube with physics
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        name="cube",
        position=np.array([0, 0, 1.0]),
        size=np.array([0.5, 0.5, 0.5]),
        color=np.array([0, 0, 1])  # Blue
    )
)

# Reset world
world.reset()

# Run simulation for 1000 steps
for i in range(1000):
    world.step(render=True)

# Cleanup
simulation_app.close()
```

**Run:**
```bash
cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1
./python.sh hello_isaac.py
```

## Importing Robots into Isaac Sim

### Method 1: Import URDF

Isaac Sim can import ROS URDF files:

1. **File → Import → URDF**
2. Select your `.urdf` file
3. Configure import settings:
   - **Fix Base Link**: Check for fixed robots
   - **Joint Drive Type**: Position/Velocity/Effort
   - **Create Physics Scene**: Auto-add physics
4. Click **Import**

Robot appears in scene with joints configured!

### Method 2: Use Pre-Built Assets

Isaac Sim includes robots:

1. **Content Browser** → Isaac → Robots
2. Drag robot (e.g., Franka, Jetbot, Carter) into scene
3. Robot is ready to use with physics

### Method 3: Python API Import

```python
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from omni.isaac.urdf import _urdf
import carb

# Import URDF
urdf_path = "/path/to/robot.urdf"
urdf_interface = _urdf.acquire_urdf_interface()

# Import settings
import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False
import_config.fix_base = True
import_config.make_default_prim = True
import_config.create_physics_scene = True

# Import
result, prim_path = urdf_interface.parse_urdf(urdf_path, import_config)
print(f"Robot imported at: {prim_path}")
```

## ROS 2 Integration

Isaac Sim has native ROS 2 support via bridges.

### Enabling ROS 2 Bridge

**Method 1: GUI**
1. Window → Extensions
2. Search "ROS2"
3. Enable **omni.isaac.ros2_bridge**

**Method 2: Python**
```python
import omni.isaac.core.utils.extensions as extensions
extensions.enable_extension("omni.isaac.ros2_bridge")
```

### Publishing ROS 2 Topics

**Example: Publish camera images**

1. Select camera in stage
2. Right-click → Create → ROS2 → Camera
3. Configure:
   - **Topic**: `/camera/image_raw`
   - **Frame ID**: `camera_link`
   - **Publish Rate**: 30 Hz
4. Play simulation → Images published!

**Verify:**
```bash
ros2 topic list | grep camera
ros2 topic echo /camera/image_raw --no-arr
```

### Subscribing to ROS 2 Topics

**Example: Subscribe to /cmd_vel**

```python
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")

import rclpy
from geometry_msgs.msg import Twist

# ROS 2 subscriber (runs in Isaac Sim context)
def cmd_vel_callback(msg: Twist):
    linear_x = msg.linear.x
    angular_z = msg.angular.z
    print(f"Received: linear={linear_x}, angular={angular_z}")
    # Apply to robot here

# Create subscriber (simplified)
# Note: Full integration requires ROS2 bridge extension
```

### Complete ROS 2 Bridge Example

```python
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
import numpy as np

# Enable ROS 2
import omni.isaac.core.utils.extensions as extensions
extensions.enable_extension("omni.isaac.ros2_bridge")

# Create world
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Add Jetbot robot (differential drive)
from omni.isaac.jetbot import Jetbot
jetbot = world.scene.add(
    Jetbot(prim_path="/World/Jetbot", name="jetbot")
)

# Add ROS 2 Differential Drive bridge
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.ros2_bridge import enable_ros2_bridge
enable_ros2_bridge()

# Create ROS 2 topics
# This is simplified - see Isaac Sim docs for full ROS2 bridge setup

world.reset()

# Simulation loop
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
```

## Week 8 Hands-On Exercise

**Task**: Create a simple pick-and-place simulation

**Requirements:**
1. Scene with ground plane and table
2. Cube object on table (target to pick)
3. Camera viewing the scene
4. Export camera feed to ROS 2 topic
5. Python script that:
   - Runs simulation for 500 steps
   - Captures camera images
   - Saves 10 frames as PNG files

**Bonus:**
- Add a robotic arm (Franka Panda from assets)
- Animate arm movement programmatically

## Common Issues

### Issue 1: Isaac Sim won't launch
**Cause**: Incompatible driver or VRAM too low
**Fix**: Update to driver 535+, close other GPU apps

### Issue 2: "CUDA out of memory"
**Fix**: Reduce scene complexity, lower resolution, close other programs

### Issue 3: ROS 2 topics not appearing
**Fix**: Source ROS 2 workspace first, enable ROS2 bridge extension

### Issue 4: Slow performance
**Fix**: Disable ray tracing (Viewport → Rendering Mode → Lit), reduce physics substeps

## Resources

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html)
- [USD Official Docs](https://openusd.org/release/index.html)
- [Isaac Sim Tutorials](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorials.html)
- [ROS 2 Bridge Guide](https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/index.html)
- [Isaac Sim Python API](https://docs.omniverse.nvidia.com/py/isaacsim/index.html)
- [NVIDIA Isaac Forum](https://forums.developer.nvidia.com/c/agx-autonomous-machines/isaac/67)

## Next Steps

Excellent work! You've set up Isaac Sim and understand the basics.

Next week: [Week 9: Synthetic Data Generation & Isaac Gym](week-09.md)

We'll explore synthetic dataset creation for training vision models and reinforcement learning with Isaac Gym!
