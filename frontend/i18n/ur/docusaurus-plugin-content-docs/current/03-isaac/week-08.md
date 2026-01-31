# ہفتہ 8: NVIDIA Isaac Sim - تعارف اور سیٹ اپ

## جائزہ

باب 3 میں خوش آمدید! یہ ہفتہ NVIDIA Isaac Sim کا تعارف پیش کرتا ہے، جو NVIDIA Omniverse پر بنایا گیا ایک GPU-accelerated robotics simulator ہے۔ آپ سیکھیں گے کہ Isaac Sim Physical AI کے لیے کیوں انقلابی ہے، پلیٹ فارم کو انسٹال کریں، اپنی پہلی simulation بنائیں، اور ROS 2 کے ساتھ integrate کریں۔

## سیکھنے کے مقاصد

اس ہفتے کے اختتام تک، آپ یہ کر سکیں گے:

- سمجھیں کہ Isaac Sim کو Gazebo سے کیا مختلف بناتا ہے
- NVIDIA Isaac Sim اور Omniverse انسٹال کریں
- Isaac Sim interface میں navigate کریں
- 3D assets (USD format) import اور manipulate کریں
- Physics کے ساتھ بنیادی robot simulations بنائیں
- Isaac Sim کو ROS 2 کے ساتھ integrate کریں
- Automation کے لیے Isaac Sim Python API استعمال کریں

## NVIDIA Isaac Sim کیا ہے؟

**Isaac Sim** ایک robotics simulation platform ہے جو NVIDIA Omniverse پر بنایا گیا ہے، اور یہ فائدے اٹھاتا ہے:

- **PhysX 5**: GPU-accelerated physics engine (CPU سے 1000x تیز)
- **RTX Ray Tracing**: Vision AI کے لیے photorealistic rendering
- **USD (Universal Scene Description)**: Industry-standard 3D format (Pixar)
- **Python API**: مکمل programmatic control
- **ROS 2 Integration**: ROS topics/services کے لیے native bridges
- **Synthetic Data Generation**: ML کے لیے خودکار dataset creation

### Isaac Sim بمقابلہ Gazebo Classic

| خصوصیت | Gazebo Classic | Isaac Sim |
|---------|---------------|-----------|
| **Physics Engine** | ODE/Bullet (CPU) | PhysX 5 (GPU) |
| **Rendering** | OGRE (بنیادی) | RTX ray tracing (photorealistic) |
| **Parallel Simulations** | محدود | GPU پر ہزاروں |
| **AI/ML Integration** | بیرونی | Native (Isaac Gym، Replicator) |
| **Sensor Simulation** | آسان کیا ہوا | جسمانی طور پر درست (cameras، lidar) |
| **Scene Format** | SDF/URDF | USD (Universal Scene Description) |
| **Extensibility** | C++ plugins | Python API |
| **License** | Open source | تحقیق/تعلیم کے لیے مفت |

**Isaac Sim کب استعمال کریں:**
- Synthetic data کے ساتھ ML models ٹریننگ کرنا
- Photorealistic vision datasets
- بہت زیادہ parallel RL (Isaac Gym)
- High-fidelity physics (deformables، fluids)
- Industrial digital twins

**Gazebo کب استعمال کریں:**
- GPU کے بغیر فوری prototyping
- Legacy ROS 1 workflows
- Open-source ضروریات
- ہلکے وزن simulations

## سسٹم کی ضروریات

### کم از کم ضروریات
- **GPU**: NVIDIA RTX 2060 یا اس سے زیادہ (6GB VRAM)
- **CPU**: Intel i7 یا AMD Ryzen 7
- **RAM**: 32GB
- **Storage**: 50GB SSD خالی جگہ
- **OS**: Ubuntu 20.04/22.04 یا Windows 10/11

### تجویز کردہ ضروریات
- **GPU**: NVIDIA RTX 3080 یا اس سے زیادہ (12GB+ VRAM)
- **CPU**: Intel i9 یا AMD Ryzen 9
- **RAM**: 64GB
- **Storage**: 100GB NVMe SSD

### Cloud اختیارات (اگر مقامی GPU نہیں ہے)
- **AWS**: g5.xlarge (A10G GPU, $1.006/hr)
- **GCP**: n1-standard-4 + T4 GPU ($0.70/hr)
- **NVIDIA Omniverse Cloud**: Streaming option (قیمت مختلف ہوتی ہے)

## NVIDIA Isaac Sim انسٹال کرنا

### مرحلہ 1: NVIDIA Driver اور CUDA

```bash
# موجودہ driver چیک کریں
nvidia-smi

# Driver انسٹال کریں (535+ تجویز کردہ)
sudo apt install nvidia-driver-535 -y
sudo reboot

# Driver تصدیق کریں
nvidia-smi
# CUDA Version: 12.2 یا اس سے زیادہ دکھانا چاہیے

# CUDA Toolkit انسٹال کریں (اختیاری، development کے لیے)
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
sudo apt install cuda-toolkit-12-2 -y
```

### مرحلہ 2: NVIDIA اکاؤنٹ بنائیں

1. [https://developer.nvidia.com/isaac-sim](https://developer.nvidia.com/isaac-sim) دیکھیں
2. "Get Started" → Sign in/Create account پر کلک کریں
3. NVIDIA Developer Program میں شامل ہوں (مفت)

### مرحلہ 3: Omniverse Launcher انسٹال کریں

**Linux:**
```bash
# Launcher ڈاؤن لوڈ کریں
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# Executable بنائیں
chmod +x omniverse-launcher-linux.AppImage

# Launcher چلائیں
./omniverse-launcher-linux.AppImage
```

**Windows:**
[https://www.nvidia.com/en-us/omniverse/download/](https://www.nvidia.com/en-us/omniverse/download/) سے installer ڈاؤن لوڈ کریں

### مرحلہ 4: Launcher کے ذریعے Isaac Sim انسٹال کریں

1. Omniverse Launcher کھولیں
2. **Exchange** ٹیب پر جائیں
3. "Isaac Sim" تلاش کریں
4. **Install** پر کلک کریں (version 2023.1.1 یا تازہ ترین منتخب کریں)
5. انسٹالیشن میں ~20-30 منٹ لگتے ہیں (20GB download)

### مرحلہ 5: Isaac Sim شروع کریں

1. Launcher میں، **Library** ٹیب پر جائیں
2. "Isaac Sim" تلاش کریں
3. **Launch** پر کلک کریں
4. پہلی شروعات میں 5-10 منٹ لگتے ہیں (shader compilation)

**تصدیق:**
- Isaac Sim window کھلنی چاہیے
- آپ کو welcome screen نظر آنی چاہیے
- Console میں کوئی error messages نہیں ہونے چاہیے

### مرحلہ 6: ROS 2 Bridge انسٹال کریں

```bash
# Isaac Sim directory میں navigate کریں
cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1

# ROS 2 install script چلائیں
./setup_python_env.sh

# ROS 2 Humble bridge انسٹال کریں
./install_ros2_humble.sh
```

## Isaac Sim Interface کا جائزہ

### اہم اجزاء

1. **Viewport**: 3D scene visualization
2. **Stage**: آبجیکٹس کی hierarchy (USD prims)
3. **Property Panel**: آبجیکٹ properties اور settings
4. **Content Browser**: Asset library
5. **Console**: Python scripts اور logs

### Navigation Controls

| عمل | کنٹرول |
|--------|---------|
| **Orbit camera** | Middle mouse drag |
| **Pan camera** | Shift + Middle mouse drag |
| **Zoom** | Mouse wheel |
| **Select object** | Left click |
| **Multi-select** | Ctrl + Left click |
| **Focus on object** | F key |
| **Frame all** | A key |

### Viewport Modes

- **Lit**: روشنیوں کے ساتھ realistic rendering
- **Wireframe**: Polygon edges دکھائیں
- **Physics Debug**: Collision shapes کو visualize کریں
- **Bounds**: Bounding boxes دکھائیں

## USD: Universal Scene Description

Isaac Sim **USD (Universal Scene Description)** استعمال کرتا ہے، Pixar کا open-source 3D format۔

### USD تصورات

**Prims (Primitives)**: Scene میں ہر چیز ایک prim ہے
- `Xform`: Transform node (position، rotation، scale)
- `Mesh`: 3D geometry
- `Material`: ظاہری خصوصیات
- `Light`: روشنی کے ذرائع
- `Camera`: نقطہ نظر

**Stage**: تمام prims کے لیے container (scene)

**Layers**: غیر تباہ کن edits (Photoshop کی تہوں کی طرح)

### GUI کے ذریعے آبجیکٹس بنانا

1. **Create → Mesh → Cube**
   - Origin پر cube بناتا ہے
   - Stage panel میں `/World/Cube` کے طور پر نظر آتا ہے

2. **Property Panel → Transform**
   - Position: X، Y، Z coordinates
   - Rotation: Euler angles یا quaternion
   - Scale: X، Y، Z scaling

3. **Property Panel → Physics**
   - **Rigid Body** component شامل کریں
   - Mass، collision shape، friction سیٹ کریں

### مثال: ایک سادہ Scene بنائیں

**کام:** اشیاء کے ساتھ میز بنائیں

```
1. Create → Mesh → Cube (میز کی سطح)
   - Scale: (2, 1, 0.1)
   - Position: (0, 0, 1)

2. Create → Mesh → Cylinder (میز کی ٹانگ)
   - Scale: (0.1, 0.1, 1)
   - Position: (0.8, 0.4, 0)
   - 4 ٹانگوں کے لیے 3 بار مزید duplicate کریں

3. Create → Mesh → Sphere (میز پر گیند)
   - Radius: 0.1
   - Position: (0, 0, 1.15)
   - Physics → Rigid Body شامل کریں

4. Create → Physics → Ground Plane

5. Physics چلانے کے لیے Play (نچلے toolbar) پر کلک کریں
```

## Isaac Sim میں Python Scripting

Isaac Sim میں مکمل API تک رسائی کے ساتھ ایک built-in Python interpreter ہے۔

### Python Scripts چلانا

**طریقہ 1: Script Editor (GUI)**
1. Window → Script Editor
2. Python code لکھیں
3. **Run** پر کلک کریں

**طریقہ 2: Standalone Python Script**
```bash
# Isaac Sim directory سے
./python.sh /path/to/script.py
```

### Hello World Script

```python
# hello_isaac.py
from omni.isaac.kit import SimulationApp

# Isaac Sim headless شروع کریں (کوئی GUI نہیں)
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
import numpy as np

# دنیا بنائیں
world = World(stage_units_in_meters=1.0)

# Ground plane شامل کریں
world.scene.add_default_ground_plane()

# Physics کے ساتھ cube شامل کریں
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        name="cube",
        position=np.array([0, 0, 1.0]),
        size=np.array([0.5, 0.5, 0.5]),
        color=np.array([0, 0, 1])  # نیلا
    )
)

# دنیا کو reset کریں
world.reset()

# 1000 steps کے لیے simulation چلائیں
for i in range(1000):
    world.step(render=True)

# Cleanup
simulation_app.close()
```

**چلائیں:**
```bash
cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1
./python.sh hello_isaac.py
```

## Robots کو Isaac Sim میں Import کرنا

### طریقہ 1: URDF Import کریں

Isaac Sim ROS URDF فائلیں import کر سکتا ہے:

1. **File → Import → URDF**
2. اپنی `.urdf` فائل منتخب کریں
3. Import settings configure کریں:
   - **Fix Base Link**: مقررہ robots کے لیے check کریں
   - **Joint Drive Type**: Position/Velocity/Effort
   - **Create Physics Scene**: خودکار طور پر physics شامل کریں
4. **Import** پر کلک کریں

Robot configured joints کے ساتھ scene میں ظاہر ہوتا ہے!

### طریقہ 2: Pre-Built Assets استعمال کریں

Isaac Sim میں robots شامل ہیں:

1. **Content Browser** → Isaac → Robots
2. Robot (مثلاً، Franka، Jetbot، Carter) کو scene میں drag کریں
3. Robot physics کے ساتھ استعمال کے لیے تیار ہے

### طریقہ 3: Python API Import

```python
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from omni.isaac.urdf import _urdf
import carb

# URDF import کریں
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

Isaac Sim میں bridges کے ذریعے native ROS 2 support ہے۔

### ROS 2 Bridge فعال کرنا

**طریقہ 1: GUI**
1. Window → Extensions
2. "ROS2" تلاش کریں
3. **omni.isaac.ros2_bridge** فعال کریں

**طریقہ 2: Python**
```python
import omni.isaac.core.utils.extensions as extensions
extensions.enable_extension("omni.isaac.ros2_bridge")
```

### ROS 2 Topics شائع کرنا

**مثال: Camera images شائع کریں**

1. Stage میں camera منتخب کریں
2. Right-click → Create → ROS2 → Camera
3. Configure کریں:
   - **Topic**: `/camera/image_raw`
   - **Frame ID**: `camera_link`
   - **Publish Rate**: 30 Hz
4. Simulation چلائیں → Images شائع ہو گئیں!

**تصدیق کریں:**
```bash
ros2 topic list | grep camera
ros2 topic echo /camera/image_raw --no-arr
```

### ROS 2 Topics Subscribe کرنا

**مثال: /cmd_vel Subscribe کریں**

```python
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")

import rclpy
from geometry_msgs.msg import Twist

# ROS 2 subscriber (Isaac Sim context میں چلتا ہے)
def cmd_vel_callback(msg: Twist):
    linear_x = msg.linear.x
    angular_z = msg.angular.z
    print(f"Received: linear={linear_x}, angular={angular_z}")
    # یہاں robot پر apply کریں

# Subscriber بنائیں (آسان کیا ہوا)
# نوٹ: مکمل integration کے لیے ROS2 bridge extension کی ضرورت ہے
```

### مکمل ROS 2 Bridge مثال

```python
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
import numpy as np

# ROS 2 فعال کریں
import omni.isaac.core.utils.extensions as extensions
extensions.enable_extension("omni.isaac.ros2_bridge")

# دنیا بنائیں
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Jetbot robot شامل کریں (differential drive)
from omni.isaac.jetbot import Jetbot
jetbot = world.scene.add(
    Jetbot(prim_path="/World/Jetbot", name="jetbot")
)

# ROS 2 Differential Drive bridge شامل کریں
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.ros2_bridge import enable_ros2_bridge
enable_ros2_bridge()

# ROS 2 topics بنائیں
# یہ آسان کیا ہوا ہے - مکمل ROS2 bridge setup کے لیے Isaac Sim docs دیکھیں

world.reset()

# Simulation loop
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
```

## ہفتہ 8 عملی مشق

**کام**: ایک سادہ pick-and-place simulation بنائیں

**ضروریات:**
1. Ground plane اور میز کے ساتھ scene
2. میز پر cube object (pick کرنے کا ہدف)
3. Scene دیکھنے والا camera
4. ROS 2 topic کو camera feed export کریں
5. Python script جو:
   - 500 steps کے لیے simulation چلاتی ہے
   - Camera images کیپچر کرتی ہے
   - 10 frames کو PNG فائلوں کے طور پر محفوظ کرتی ہے

**بونس:**
- Robotic arm شامل کریں (assets سے Franka Panda)
- Programmatically arm کی حرکت کو animate کریں

## عام مسائل

### مسئلہ 1: Isaac Sim شروع نہیں ہوتا
**وجہ**: غیر مطابقت پذیر driver یا VRAM بہت کم
**حل**: Driver 535+ پر اپ ڈیٹ کریں، دوسری GPU ایپس بند کریں

### مسئلہ 2: "CUDA out of memory"
**حل**: Scene کی پیچیدگی کم کریں، resolution کم کریں، دوسرے programs بند کریں

### مسئلہ 3: ROS 2 topics ظاہر نہیں ہو رہے
**حل**: پہلے ROS 2 workspace source کریں، ROS2 bridge extension فعال کریں

### مسئلہ 4: سست کارکردگی
**حل**: Ray tracing غیر فعال کریں (Viewport → Rendering Mode → Lit)، physics substeps کم کریں

## وسائل

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html)
- [USD Official Docs](https://openusd.org/release/index.html)
- [Isaac Sim Tutorials](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorials.html)
- [ROS 2 Bridge Guide](https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/index.html)
- [Isaac Sim Python API](https://docs.omniverse.nvidia.com/py/isaacsim/index.html)
- [NVIDIA Isaac Forum](https://forums.developer.nvidia.com/c/agx-autonomous-machines/isaac/67)

## اگلے قدم

بہترین کام! آپ نے Isaac Sim سیٹ اپ کر لیا ہے اور بنیادی باتیں سمجھ گئے ہیں۔

اگلا ہفتہ: [ہفتہ 9: Synthetic Data Generation & Isaac Gym](week-09.md)

ہم vision models ٹریننگ کرنے کے لیے synthetic dataset creation اور Isaac Gym کے ساتھ reinforcement learning کو دریافت کریں گے!

---

## 📝 ہفتہ وار Quiz

اس ہفتے کے مواد کی اپنی سمجھ کو جانچیں! Quiz multiple choice ہے، خودکار طور پر score ہوتا ہے، اور آپ کے پاس 2 کوششیں ہیں۔

**[ہفتہ 8 Quiz لیں →](/quiz?week=8)**
