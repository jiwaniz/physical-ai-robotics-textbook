# ہفتہ 3: آر او ایس 2 آرکیٹیکچر اور بنیادی تصورات

## جائزہ

باب 1 میں خوش آمدید! اس ہفتے آر او ایس 2 (روبوٹ آپریٹنگ سسٹم 2) متعارف کرایا جاتا ہے، جو ماڈیولر، ڈسٹریبیوٹڈ روبوٹک سسٹمز بنانے کے لیے انڈسٹری اسٹینڈرڈ مڈل ویئر ہے۔ آپ بنیادی آرکیٹیکچر سیکھیں گے، آر او ایس 2 ہمبل انسٹال کریں گے، اور اپنا پہلا آر او ایس 2 نوڈ بنائیں گے۔

## سیکھنے کے مقاصد

اس ہفتے کے اختتام تک، آپ یہ کر سکیں گے:

- آر او ایس 2 کیا ہے اور جدید روبوٹکس کے لیے یہ کیوں ضروری ہے، وضاحت کرنا
- آر او ایس 2 آرکیٹیکچر (نوڈز، ٹاپکس، ڈی ڈی ایس) کو سمجھنا
- اوبنٹو 22.04 پر آر او ایس 2 ہمبل انسٹال کرنا
- ایک سادہ آر او ایس 2 پائتھون نوڈ بنانا اور چلانا
- بنیادی آر او ایس 2 کمانڈ لائن ٹولز استعمال کرنا (`ros2 node`، `ros2 topic`، `ros2 run`)
- آر او ایس 2 ورک سپیسز اور پیکیج اسٹرکچر میں نیویگیٹ کرنا

## آر او ایس 2 کیا ہے؟

**آر او ایس 2 (روبوٹ آپریٹنگ سسٹم 2)** کوئی آپریٹنگ سسٹم نہیں، بلکہ ایک **مڈل ویئر فریم ورک** ہے جو فراہم کرتا ہے:

- **کمیونیکیشن انفراسٹرکچر**: اجزاء کے درمیان میسج پاسنگ
- **ہارڈویئر ایبسٹریکشن**: سینسرز/ایکچویٹرز کے لیے یکساں انٹرفیسز
- **پیکیج مینجمنٹ**: ماڈیولر، دوبارہ استعمال کے قابل سافٹ ویئر کمپوننٹس
- **بلڈ سسٹم**: پیچیدہ پراجیکٹس کو کمپائل اور مینیج کرنا
- **ٹولنگ ایکو سسٹم**: ویژولائزیشن (آر ویز)، سمیولیشن (گزیبو)، ڈیبگنگ

### آر او ایس 1 بمقابلہ آر او ایس 2: اپ گریڈ کیوں؟

| فیچر | آر او ایس 1 (2007-2020) | آر او ایس 2 (2017-موجودہ) |
|---------|-------------------|----------------------|
| **کمیونیکیشن** | کسٹم TCPROS/UDPROS | ڈی ڈی ایس (انڈسٹری اسٹینڈرڈ) |
| **ریئل ٹائم سپورٹ** | محدود | ہاں (ریئل ٹائم او ایس کے ساتھ) |
| **سیکیورٹی** | کوئی نہیں | آتھینٹیکیشن، انکرپشن |
| **ملٹی روبوٹ** | مشکل | نیٹو سپورٹ |
| **ایمبیڈڈ سسٹمز** | نہیں | ہاں (مائیکرو-آر او ایس) |
| **لائف سائیکل مینجمنٹ** | بنیادی | مینیجڈ نوڈز |
| **کیو او ایس (کوالٹی آف سروس)** | کوئی نہیں | کنفیگریبل ریلائبلٹی |
| **پلیٹ فارم سپورٹ** | صرف لینکس | لینکس، ونڈوز، میک او ایس |

**آر او ایس 2 میں اہم بہتریاں:**
- تجارتی روبوٹس کے لیے پروڈکشن ریڈی
- حفاظتی لحاظ سے اہم نظاموں کے لیے ریئل ٹائم قابل
- نیٹ ورک شدہ روبوٹس کے لیے بہتر سیکیورٹی
- زیادہ لچکدار کمیونیکیشن پیٹرنز

## آر او ایس 2 آرکیٹیکچر

### 1. نوڈز: بلڈنگ بلاکس

ایک **نوڈ** ایک پروسیس ہے جو ایک مخصوص کام انجام دیتا ہے (مثلاً، کیمرا پڑھنا، پاتھ پلان کرنا، موٹر کنٹرول کرنا)۔ نوڈز، آر او ایس 2 میں کمپیوٹیشن کی بنیادی اکائی ہیں۔

**اہم خصوصیات:**
- ماڈیولر: ہر نوڈ ایک کام اچھی طرح کرتا ہے
- ڈسٹریبیوٹڈ: نوڈز مختلف مشینز پر چل سکتے ہیں
- لینگویج ایگناسٹک: پائتھون، سی++، یا رسٹ میں لکھیں
- لائف سائیکل-مینیجڈ: شائستگی سے اسٹارٹ، پاز، اسٹاپ

**نوڈ ذمہ داریوں کی مثال:**
- `camera_driver`: کیمرا سے امیجز کیپچر کرنا
- `object_detector`: امیجز میں آبجیکٹس ڈیٹیکٹ کرنا
- `motion_planner`: کولیژن-فری پاتھز پلان کرنا
- `motor_controller`: موٹرز کو کمانڈز بھیجنا

### 2. ٹاپکس: اسینکرونس میسج پاسنگ

**ٹاپکس** پبلش-سبسکرائب کمیونیکیشن کو قابل بناتے ہیں:

```
┌──────────────┐         /camera/image          ┌──────────────┐
│   Camera     │────────────────────────────────▶│   Object     │
│   Driver     │      (Image messages)           │   Detector   │
└──────────────┘                                 └──────────────┘
    Publisher                                       Subscriber
```

**خصوصیات:**
- **مینی-ٹو-مینی**: متعدد پبلشرز، متعدد سبسکرائبرز
- **اسینکرونس**: جواب کا انتظار نہیں
- **ٹائپڈ**: میسجز کی متعین ساخت ہے (مثلاً، `sensor_msgs/Image`)
- **بفرڈ**: کیو او ایس پالیسیز میسج کیو کے رویے کو کنٹرول کرتی ہیں

**استعمال کا معاملہ**: سینسر ڈیٹا سٹریمز (کیمرا، لائیڈار، آئی ایم یو)

### 3. سروسز: سینکرونس ریکویسٹ-رسپانس

**سروسز** کلائنٹ-سرور کمیونیکیشن کو قابل بناتی ہیں:

```
┌──────────────┐      Request: "Plan path      ┌──────────────┐
│   Navigation │      from A to B"             │    Motion    │
│    Node      │──────────────────────────────▶│   Planner    │
│              │◀──────────────────────────────│              │
└──────────────┘      Response: [waypoints]    └──────────────┘
    Client                                          Server
```

**خصوصیات:**
- **ون-ٹو-ون**: ایک کلائنٹ، ایک سرور
- **سینکرونس**: کلائنٹ جواب کا انتظار کرتا ہے
- **ٹائپڈ**: ریکویسٹ اور رسپانس کی متعین ساخت ہے

**استعمال کا معاملہ**: کبھی کبھار کی کمپیوٹیشنز (پاتھ پلاننگ، آبجیکٹ ریکگنیشن)

### 4. ایکشنز: طویل چلنے والے کام فیڈ بیک کے ساتھ

**ایکشنز** ایسے کاموں کے لیے سروسز کو بڑھاتے ہیں جو وقت لیتے ہیں:

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
- **فیڈ بیک**: ایگزیکیوشن کے دوران پروگریس اپڈیٹس
- **کینسل ایبل**: کلائنٹ گول کو کینسل کر سکتا ہے
- **پری ایمپٹ ایبل**: نئے گولز پرانے کو اوور رائیڈ کر سکتے ہیں

**استعمال کا معاملہ**: روبوٹ موشنز، گراسپنگ، نیویگیشن

### 5. پیرامیٹرز: رن ٹائم کنفیگریشن

**پیرامیٹرز** کنفیگریشن ویلیوز کو اسٹور کرتے ہیں جو ری کمپائلنگ کے بغیر تبدیل کیے جا سکتے ہیں:

```python
# Declare parameter with default value
self.declare_parameter('max_speed', 1.0)

# Get parameter value
max_speed = self.get_parameter('max_speed').value

# Set parameter from command line
ros2 run my_package my_node --ros-args -p max_speed:=2.5
```

**استعمال کا معاملہ**: ٹیوننگ، کیلیبریشن، انوائرمنٹ-اسپیسیفک سیٹنگز

### 6. ڈی ڈی ایس: کمیونیکیشن لیئر

آر او ایس 2 **ڈی ڈی ایس (ڈیٹا ڈسٹریبیوشن سروس)** استعمال کرتا ہے، ایک پختہ مڈل ویئر اسٹینڈرڈ:

**فوائد:**
- انڈسٹری پروون (ایروسپیس، ڈیفنس، آٹوموٹو)
- خودکار ڈسکوری (آر او ایس 1 جیسا ماسٹر نوڈ نہیں)
- کیو او ایس پالیسیز (ریلائبلٹی، ڈیورایبلٹی، لیٹنسی)
- سیکیورٹی (آتھینٹیکیشن، انکرپشن)

**ڈی ڈی ایس امپلیمنٹیشنز:**
- فاسٹ ڈی ڈی ایس (ڈیفالٹ، ایپروسیما)
- سائیکلون ڈی ڈی ایس (ایکلپس)
- کونیکٹ ڈی ڈی ایس (آر ٹی آئی، کمرشل)

## آر او ایس 2 ہمبل انسٹال کرنا

آر او ایس 2 ہمبل ہاکسبل ایک ایل ٹی ایس (لانگ-ٹرم سپورٹ) ریلیز ہے جو مئی 2027 تک سپورٹڈ ہے۔

### قدم 1: لوکیل سیٹ کریں

```bash
locale  # Check current settings
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### قدم 2: آر او ایس 2 ریپوزٹری شامل کریں

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

### قدم 3: آر او ایس 2 ہمبل انسٹال کریں

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

**تنصیب میں تقریباً 10 منٹ اور 2 جی بی ڈسک جگہ لگتی ہے۔**

### قدم 4: آر او ایس 2 سیٹ اپ کو سورس کریں

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

**متوقع آؤٹ پٹ:**
```
Terminal 1:
[INFO] [talker]: Publishing: 'Hello World: 1'
[INFO] [talker]: Publishing: 'Hello World: 2'

Terminal 2:
[INFO] [listener]: I heard: [Hello World: 1]
[INFO] [listener]: I heard: [Hello World: 2]
```

اگر آپ نوڈز کے درمیان میسجز دیکھتے ہیں، تو آر او ایس 2 کام کر رہا ہے!

## اپنا پہلا آر او ایس 2 نوڈ بنانا

آئیے ایک سادہ "ہیلو روبوٹ" نوڈ بنائیں۔

### قدم 1: ورک سپیس بنائیں

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### قدم 2: پیکیج بنائیں

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

### قدم 3: نوڈ اسکرپٹ بنائیں

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

### قدم 4: سیٹ اپ فائلز کو اپ ڈیٹ کریں

**`setup.py` میں ترمیم کریں** - انٹری پوائنٹ شامل کریں:

```python
entry_points={
    'console_scripts': [
        'hello_node = hello_robot_py.hello_node:main',
    ],
},
```

### قدم 5: پیکیج کو بلڈ کریں

```bash
# Navigate to workspace root
cd ~/ros2_ws

# Build package
colcon build --packages-select hello_robot_py

# Source workspace overlay
source install/setup.bash
```

### قدم 6: اپنا نوڈ چلائیں

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

**مبارک ہو! آپ نے اپنا پہلا آر او ایس 2 نوڈ بنا لیا!** 🎉

## ضروری آر او ایس 2 کمانڈ لائن ٹولز

### نوڈ کمانڈز
```bash
ros2 node list                    # List running nodes
ros2 node info /node_name         # Show node details
```

### ٹاپک کمانڈز
```bash
ros2 topic list                   # List active topics
ros2 topic echo /topic_name       # Print messages
ros2 topic hz /topic_name         # Show publishing rate
ros2 topic info /topic_name       # Show publishers/subscribers
ros2 topic pub /topic_name ...    # Publish message from CLI
```

### پیرامیٹر کمانڈز
```bash
ros2 param list                   # List parameters
ros2 param get /node_name param   # Get parameter value
ros2 param set /node_name param value  # Set parameter
```

### عمومی کمانڈز
```bash
ros2 pkg list                     # List installed packages
ros2 interface show Type          # Show message definition
ros2 doctor                       # Check ROS 2 setup
```

## آر او ایس 2 ورک سپیس اسٹرکچر

```
ros2_ws/                      # Workspace root
├── src/                      # Source code
│   └── hello_robot_py/       # Your package
├── build/                    # Build artifacts (auto-generated)
├── install/                  # Installed packages (auto-generated)
└── log/                      # Build logs (auto-generated)
```

**بہترین طریقے:**
- صرف `src/` کو ورژن کنٹرول میں کمٹ کریں
- `build/`، `install/`، `log/` کو `.gitignore` میں شامل کریں
- مختلف پراجیکٹس کے لیے الگ ورک سپیسز استعمال کریں

## آر او ایس 2 پیکیجز کو سمجھنا

ایک **پیکیج** آر او ایس 2 میں بلڈ اور ریلیز کی سب سے چھوٹی اکائی ہے۔

**پیکیج کے اجزاء:**
- `package.xml`: میٹا ڈیٹا (نام، ورژن، ڈیپنڈنسیز)
- `CMakeLists.txt` (سی++) یا `setup.py` (پائتھون): بلڈ کنفیگریشن
- سورس کوڈ: نوڈ امپلیمنٹیشنز
- لانچ فائلز: متعدد نوڈز شروع کرنا
- کنفیگ فائلز: پیرامیٹرز، یو آر ڈی ایف ماڈلز

**پیکیج کی اقسام:**
- **ament_python**: خالص پائتھون پیکیجز
- **ament_cmake**: سی++ پیکیجز یا مکسڈ
- **ament_cmake_python**: پائتھون نوڈز کے ساتھ سی++

## عام مسائل اور حل

### مسئلہ 1: بلڈنگ کے بعد "پیکیج ناٹ فاؤنڈ"
**وجہ**: ورک سپیس کو سورس کرنا بھول گئے
**حل**: `source ~/ros2_ws/install/setup.bash`

### مسئلہ 2: نوڈ کو میسجز موصول نہیں ہوتے
**وجہ**: پبلشر/سبسکرائبر کے درمیان کیو او ایس مسمیچ
**حل**: یقینی بنائیں کہ دونوں کمپیٹیبل کیو او ایس سیٹنگز استعمال کرتے ہیں (اگلے ہفتے کور کیا جائے گا)

### مسئلہ 3: "colcon: command not found"
**وجہ**: آر او ایس 2 ڈیو ٹولز انسٹال نہیں
**حل**: `sudo apt install python3-colcon-common-extensions`

### مسئلہ 4: پائتھون نوڈ چلاتے وقت امپورٹ ایرر
**وجہ**: پیکیج صحیح طریقے سے انسٹال نہیں
**حل**: `colcon build --symlink-install` کے ساتھ دوبارہ بلڈ کریں

## ہفتہ 3 کی عملی مشق

**کام**: ہیلو روبوٹ نوڈ میں ترمیم کریں تاکہ:
1. ایک پیرامیٹر `robot_name` قبول کرے (ڈیفالٹ: "فزکس بوٹ")
2. روبوٹ کا نام اسٹیٹس میسجز میں شامل کرے
3. قابل تشکیل ریٹ پر پبلش کرے (پیرامیٹر `publish_rate`، ڈیفالٹ: 1.0 ہرٹز)

**بونس**: ایک دوسرا نوڈ بنائیں جو `/robot_status` کو سبسکرائب کرے اور موصول شدہ میسجز کو لاگ کرے۔

**جمع کرانا**: کوڈ کو گٹ ہب پر پش کریں اور ریپوزٹری لنک شیئر کریں۔

## کوئز کے سوالات

1. آر او ایس 2 میں کمپیوٹیشن کی بنیادی اکائی کیا ہے؟
2. ٹاپکس اور سروسز کے درمیان فرق کی وضاحت کریں۔
3. آر او ایس 2 نے کسٹم پروٹوکولز کے بجائے ڈی ڈی ایس کیوں اپنایا؟
4. کون سا کمانڈ تمام فعال ٹاپکس دکھاتا ہے؟
5. `colcon build` کا مقصد کیا ہے؟

## اگلے قدم

ہفتہ 3 مکمل کرنے پر بہترین کام! اب آپ آر او ایس 2 آرکیٹیکچر کو سمجھتے ہیں اور آپ کے پاس ایک کام کرنے والا ڈیولپمنٹ انوائرمنٹ ہے۔

اگلا ہفتہ: [ہفتہ 4: نوڈز، ٹاپکس، پبلشرز اور سبسکرائبرز](week-04.md)

ہم پب-سب کمیونیکیشن میں گہرائی سے جائیں گے، میسج ٹائپس، اور ایک ملٹی نوڈ روبوٹ سسٹم بنائیں گے!

## وسائل

- [آر او ایس 2 ہمبل ڈاکیومینٹیشن](https://docs.ros.org/en/humble/)
- [آر او ایس 2 ٹیوٹوریلز](https://docs.ros.org/en/humble/Tutorials.html)
- [آر او ایس 2 نوڈز کو سمجھنا](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
- [آر او ایس 2 ڈیزائن ڈاکیومینٹس](https://design.ros2.org/)
- [ڈی ڈی ایس سپیسیفکیشن](https://www.omg.org/spec/DDS/)

---

## 📝 ہفتہ وار کوئز

اس ہفتے کے مواد کی اپنی سمجھ کو جانچیں! کوئز ملٹیپل چوائس ہے، خودکار طریقے سے اسکور کیا جاتا ہے، اور آپ کے پاس 2 کوششیں ہیں۔

**[ہفتہ 3 کوئز لیں →](/quiz?week=3)**
