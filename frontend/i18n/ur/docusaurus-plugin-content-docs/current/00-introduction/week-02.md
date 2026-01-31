# ہفتہ 2: Development Environment سیٹ اپ

## جائزہ

یہ ہفتہ کورس کے لیے آپ کے ڈیولپمنٹ ماحول کو تیار کرنے پر مرکوز ہے۔ آپ Ubuntu 22.04 انسٹال کریں گے (ROS 2 Humble کے لیے معیار)، ضروری ٹولز سیٹ اپ کریں گے، اور ایک سادہ "Hello World" پروجیکٹ کے ساتھ اپنی تنصیب کی تصدیق کریں گے۔ مناسب ماحول کا سیٹ اپ اب بعد میں debugging کے گھنٹے بچائے گا!

## سیکھنے کے مقاصد

اس ہفتے کے اختتام تک، آپ قابل ہوں گے:

- Ubuntu 22.04 LTS انسٹال کریں (native، dual-boot، WSL2، یا VM)
- ضروری ڈیولپمنٹ ٹولز کنفیگر کریں (Python، Git، VS Code)
- Linux کی بنیادی باتیں سمجھیں (terminal، package management، file permissions)
- Containerized ماحول کے لیے Docker انسٹال کریں
- سادہ روبوٹکس "Hello World" کے ساتھ اپنے سیٹ اپ کی تصدیق کریں

## Ubuntu 22.04 تنصیب

ROS 2 Humble (اس کورس میں استعمال شدہ ورژن) سرکاری طور پر **Ubuntu 22.04 LTS (Jammy Jellyfish)** کو سپورٹ کرتا ہے۔ وہ تنصیب کا طریقہ منتخب کریں جو آپ کے لیے بہترین کام کرے:

### آپشن 1: Native تنصیب (تجویز کردہ)

**بہترین برائے**: زیادہ سے زیادہ کارکردگی، GPU رسائی، real-time صلاحیتیں

**ضروریات**: مخصوص مشین یا dual-boot سیٹ اپ

**اقدامات**:
1. [ubuntu.com/download](https://ubuntu.com/download/desktop) سے Ubuntu 22.04 Desktop ISO ڈاؤن لوڈ کریں
2. [Rufus](https://rufus.ie/) (Windows) یا [Etcher](https://www.balena.io/etcher/) (Mac/Linux) کے ساتھ bootable USB بنائیں
3. USB سے boot کریں اور installation wizard کی پیروی کریں
4. Dual-boot کے لیے "Install alongside Windows" یا مخصوص مشین کے لیے "Erase disk" منتخب کریں
5. صارف کا اکاؤنٹ بنائیں اور مضبوط پاس ورڈ سیٹ کریں

**Post-install**:
```bash
# سسٹم پیکجز کو اپ ڈیٹ کریں
sudo apt update && sudo apt upgrade -y

# ضروری build ٹولز انسٹال کریں
sudo apt install build-essential git curl wget vim -y
```

### آپشن 2: WSL2 (Windows Subsystem for Linux)

**بہترین برائے**: Windows صارفین جو dual-boot کے بغیر Linux چاہتے ہیں

**ضروریات**: Windows 10 ورژن 2004+ یا Windows 11

**اقدامات**:
```bash
# PowerShell میں Administrator کے طور پر چلائیں
wsl --install -d Ubuntu-22.04

# تنصیب کے بعد، Start Menu سے Ubuntu 22.04 شروع کریں
# پوچھے جانے پر username اور password بنائیں

# WSL2 کے اندر، پیکجز کو اپ ڈیٹ کریں
sudo apt update && sudo apt upgrade -y
```

**GPU سپورٹ (Isaac Sim کے لیے)**:
- [NVIDIA CUDA on WSL2](https://docs.nvidia.com/cuda/wsl-user-guide/index.html) انسٹال کریں
- Windows host پر NVIDIA driver 510.39.01+ کی ضرورت ہے

**حدود**:
- ڈیفالٹ طور پر کوئی GUI نہیں (X11 forwarding یا VcXsrv استعمال کریں)
- USB device passthrough محدود ہے
- Native سے تھوڑا سست

### آپشن 3: Virtual Machine (VirtualBox/VMware)

**بہترین برائے**: ٹیسٹنگ، سیکھنا، کم وابستگی

**ضروریات**: 8GB+ RAM والی Host مشین، BIOS میں virtualization فعال

**اقدامات** (VirtualBox مثال):
1. [VirtualBox](https://www.virtualbox.org/) انسٹال کریں
2. Ubuntu 22.04 Desktop ISO ڈاؤن لوڈ کریں
3. نیا VM بنائیں: 4 CPU cores، 8GB RAM، 60GB dynamic disk
4. ISO mount کریں اور Ubuntu انسٹال کریں
5. بہتر کارکردگی کے لیے VirtualBox Guest Additions انسٹال کریں

**حدود**:
- کوئی GPU passthrough نہیں (NVIDIA Isaac Sim سپورٹ نہیں)
- Gazebo اور ہلکے simulations تک محدود
- کارکردگی کا overhead

### آپشن 4: Cloud Instance (AWS/GCP/Azure)

**بہترین برائے**: کوئی مقامی ہارڈویئر نہیں، طاقتور GPU کی ضرورت، عارضی استعمال

**تجویز کردہ instances**:
- **AWS**: g4dn.xlarge (T4 GPU، $0.526/hr)
- **GCP**: n1-standard-4 + T4 GPU ($0.35/hr + $0.35/hr)
- **Azure**: NC4as_T4_v3 (T4 GPU، $0.526/hr)

**سیٹ اپ**:
1. Ubuntu 22.04 LTS AMI/image منتخب کریں
2. Security group کنفیگر کریں (SSH port 22، اختیاری طور پر VNC port 5900)
3. Instance میں SSH کریں: `ssh -i key.pem ubuntu@<ip-address>`
4. اگر ضرورت ہو تو desktop environment انسٹال کریں: `sudo apt install ubuntu-desktop`

**لاگت کا انتظام**:
- استعمال میں نہ ہونے پر instance بند کریں
- Spot/preemptible instances استعمال کریں (70% رعایت)
- Billing alerts سیٹ کریں

## ضروری ڈیولپمنٹ ٹولز

### 1. Python 3.11+ سیٹ اپ

Ubuntu 22.04 Python 3.10 کے ساتھ آتا ہے۔ بہتر کارکردگی کے لیے 3.11 میں اپ گریڈ کریں:

```bash
# Python 3.11 کے لیے deadsnakes PPA شامل کریں
sudo add-apt-repository ppa:deadsnakes/ppa -y
sudo apt update

# Python 3.11 اور ٹولز انسٹال کریں
sudo apt install python3.11 python3.11-venv python3.11-dev python3-pip -y

# تنصیب کی تصدیق کریں
python3.11 --version  # Python 3.11.x دکھانا چاہیے

# Python 3.11 کو default کے طور پر سیٹ کریں (اختیاری)
sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.11 1

# Dependency management کے لیے pipenv یا poetry انسٹال کریں
pip3 install pipenv poetry
```

### 2. Git Configuration

```bash
# Git انسٹال کریں
sudo apt install git -y

# شناخت کنفیگر کریں
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"

# Default branch نام سیٹ کریں
git config --global init.defaultBranch main

# Credential caching فعال کریں (بار بار password داخل کرنے سے بچیں)
git config --global credential.helper cache

# Configuration کی تصدیق کریں
git config --list
```

### 3. VS Code تنصیب

**طریقہ 1: Snap (تجویز کردہ)**
```bash
sudo snap install code --classic
```

**طریقہ 2: .deb Package**
```bash
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" | sudo tee /etc/apt/sources.list.d/vscode.list
sudo apt update
sudo apt install code -y
```

**تجویز کردہ Extensions**:
- Python (Microsoft)
- Pylance
- ROS (Microsoft)
- CMake Tools
- Docker
- GitLens

Command line کے ذریعے انسٹال کریں:
```bash
code --install-extension ms-python.python
code --install-extension ms-python.vscode-pylance
code --install-extension ms-iot.vscode-ros
code --install-extension ms-vscode.cmake-tools
code --install-extension ms-azuretools.vscode-docker
code --install-extension eamodio.gitlens
```

### 4. Docker تنصیب

Docker قابل تکرار ماحول کے لیے ضروری ہے اور باب 3-4 میں استعمال کیا جائے گا۔

```bash
# Dependencies انسٹال کریں
sudo apt install ca-certificates curl gnupg lsb-release -y

# Docker کی سرکاری GPG key شامل کریں
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

# Repository سیٹ اپ کریں
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Docker Engine انسٹال کریں
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin -y

# اپنے user کو docker group میں شامل کریں (docker commands کے لیے sudo سے بچیں)
sudo usermod -aG docker $USER

# Group تبدیلیوں کے اثر کے لیے log out اور واپس log in کریں
# یا چلائیں: newgrp docker

# تنصیب کی تصدیق کریں
docker --version
docker run hello-world
```

### 5. NVIDIA GPU سیٹ اپ (اگر قابل اطلاق ہو)

NVIDIA GPUs والے صارفین کے لیے (باب 3 میں Isaac Sim کے لیے ضروری):

```bash
# GPU چیک کریں
lspci | grep -i nvidia

# NVIDIA drivers انسٹال کریں
sudo apt install nvidia-driver-535 -y  # یا تازہ ترین مستحکم ورژن
sudo reboot

# Driver تنصیب کی تصدیق کریں
nvidia-smi  # GPU کی معلومات دکھانی چاہیے

# NVIDIA Container Toolkit انسٹال کریں (Docker GPU سپورٹ کے لیے)
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt update
sudo apt install nvidia-container-toolkit -y

# Docker کو NVIDIA runtime استعمال کرنے کے لیے کنفیگر کریں
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Docker میں GPU ٹیسٹ کریں
docker run --rm --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi
```

## Linux Command Line ضروری باتیں

اگر آپ Linux میں نئے ہیں، تو یہ بنیادی باتیں سیکھیں:

### Navigation اور File Management
```bash
pwd                    # موجودہ directory کو print کریں
ls -lah                # فائلیں list کریں (تفصیلی، hidden سمیت)
cd /path/to/directory  # Directory تبدیل کریں
cd ~                   # Home directory میں جائیں
cd ..                  # ایک سطح اوپر جائیں

mkdir my_project       # Directory بنائیں
touch file.txt         # خالی فائل بنائیں
cp source dest         # فائل کاپی کریں
mv old new             # فائل move/rename کریں
rm file.txt            # فائل حذف کریں
rm -rf directory/      # Directory کو recursively حذف کریں
```

### File Permissions
```bash
chmod +x script.sh     # فائل کو executable بنائیں
chmod 644 file.txt     # Permissions سیٹ کریں (owner read/write، دیگر read)
chown user:group file  # Ownership تبدیل کریں
```

### Package Management
```bash
sudo apt update                  # Package lists کو اپ ڈیٹ کریں
sudo apt upgrade                 # انسٹال شدہ packages کو اپ گریڈ کریں
sudo apt install <package>       # Package انسٹال کریں
sudo apt remove <package>        # Package ہٹائیں
sudo apt search <keyword>        # Packages تلاش کریں
```

### Process Management
```bash
ps aux                 # تمام processes کی فہرست
top                    # Interactive process monitor
htop                   # بہتر process monitor (انسٹال: sudo apt install htop)
kill <PID>             # ID کے ذریعے process کو kill کریں
killall <name>         # نام کے ذریعے processes کو kill کریں
```

### Text Editing
```bash
nano file.txt          # سادہ text editor
vim file.txt           # جدید editor (سیکھنے کا curve!)
code file.txt          # VS Code میں کھولیں
```

## تصدیقی "Hello World" پروجیکٹ

آئیے ایک سادہ Python پروجیکٹ کے ساتھ اپنے سیٹ اپ کی تصدیق کریں:

### قدم 1: پروجیکٹ ڈائریکٹری بنائیں
```bash
mkdir -p ~/robotics_hello_world
cd ~/robotics_hello_world
```

### قدم 2: Virtual Environment بنائیں
```bash
python3.11 -m venv venv
source venv/bin/activate  # Virtual environment کو فعال کریں
```

### قدم 3: Python Script بنائیں
```bash
code hello_robot.py  # یا nano/vim استعمال کریں
```

درج ذیل کوڈ شامل کریں:
```python
#!/usr/bin/env python3
"""
Hello World for Robotics - Simulated Robot State
"""
import time
import random

class SimpleRobot:
    def __init__(self, name):
        self.name = name
        self.position = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.battery = 100.0

    def move(self, dx, dy):
        self.position["x"] += dx
        self.position["y"] += dy
        self.battery -= 0.5
        print(f"{self.name} moved to ({self.position['x']:.2f}, {self.position['y']:.2f})")

    def rotate(self, dtheta):
        self.position["theta"] += dtheta
        self.battery -= 0.2
        print(f"{self.name} rotated to {self.position['theta']:.2f} rad")

    def status(self):
        print(f"\n{'='*40}")
        print(f"Robot: {self.name}")
        print(f"Position: ({self.position['x']:.2f}, {self.position['y']:.2f})")
        print(f"Orientation: {self.position['theta']:.2f} rad")
        print(f"Battery: {self.battery:.1f}%")
        print(f"{'='*40}\n")

def main():
    print("Physical AI Course - Hello World Robot Simulation\n")

    robot = SimpleRobot("PhysicsBot-001")
    robot.status()

    # سادہ حرکات کی نقل کریں
    commands = [
        ("move", 1.0, 0.0),
        ("move", 0.0, 1.0),
        ("rotate", 0.785),  # 45 degrees
        ("move", 0.5, 0.5),
    ]

    for cmd in commands:
        if cmd[0] == "move":
            robot.move(cmd[1], cmd[2])
        elif cmd[0] == "rotate":
            robot.rotate(cmd[1])
        time.sleep(0.5)  # حقیقی وقت کی تاخیر کی نقل کریں

    robot.status()
    print("✅ Hello World simulation مکمل!")

if __name__ == "__main__":
    main()
```

### قدم 4: Script چلائیں
```bash
chmod +x hello_robot.py
python3 hello_robot.py
```

**متوقع Output**:
```
Physical AI Course - Hello World Robot Simulation

========================================
Robot: PhysicsBot-001
Position: (0.00, 0.00)
Orientation: 0.00 rad
Battery: 100.0%
========================================

PhysicsBot-001 moved to (1.00, 0.00)
PhysicsBot-001 moved to (1.00, 1.00)
PhysicsBot-001 rotated to 0.79 rad
PhysicsBot-001 moved to (1.50, 1.50)

========================================
Robot: PhysicsBot-001
Position: (1.50, 1.50)
Orientation: 0.79 rad
Battery: 97.7%
========================================

✅ Hello World simulation مکمل!
```

### قدم 5: Version Control
```bash
git init
git add hello_robot.py
git commit -m "Initial commit: Hello World robot simulation"
```

## عام مسائل کا حل

### مسئلہ 1: "python3.11: command not found"
**حل**: Python 3.11 انسٹال نہیں ہے۔ Python تنصیب کے حصے پر دوبارہ جائیں۔

### مسئلہ 2: Docker چلاتے وقت "Permission denied"
**حل**: User docker group میں نہیں ہے۔ `sudo usermod -aG docker $USER` چلائیں اور log out/in کریں۔

### مسئلہ 3: nvidia-smi "NVIDIA-SMI has failed" دکھاتا ہے
**حل**: Driver انسٹال نہیں ہے یا incompatible ہے۔ `sudo apt install nvidia-driver-535` چلائیں اور reboot کریں۔

### مسئلہ 4: VS Code extensions انسٹال نہیں ہو رہے
**حل**: انٹرنیٹ کنکشن چیک کریں۔ Extensions marketplace سے manually انسٹال کرنے کی کوشش کریں۔

### مسئلہ 5: VM کی سست کارکردگی
**حل**: RAM/CPU allocation بڑھائیں، BIOS میں hardware virtualization فعال کریں، guest additions انسٹال کریں۔

## ہفتہ 2 Quiz اور تشخیص

اپنے ماحول کے سیٹ اپ کے علم کو جانچیں:

1. ROS 2 Humble کے لیے سرکاری طور پر supported Ubuntu ورژن کیا ہے؟
2. اس کورس کے لیے Ubuntu 24.04 کے مقابلے میں Ubuntu 22.04 LTS کیوں ترجیح دی جاتی ہے؟
3. Python virtual environment کا مقصد کیا ہے؟
4. آپ Linux میں یہ کیسے چیک کرتے ہیں کہ آیا آپ کا NVIDIA GPU detect ہو رہا ہے؟
5. `apt update` اور `apt upgrade` میں کیا فرق ہے؟

**ہاتھوں سے تشخیص**:
- "Hello World" robot script کامیابی سے چلائیں
- GitHub repository بنائیں اور اپنا hello_robot.py push کریں
- `nvidia-smi` output کا screenshot لیں (صرف GPU صارفین)
- ROS extension کے ساتھ انسٹال شدہ VS Code کا screenshot جمع کرائیں

## اگلے اقدامات

مبارک ہو! آپ کا ڈیولپمنٹ ماحول تیار ہے۔ اگلے ہفتے، آپ **ROS 2 بنیادی باتوں** میں غوطہ لگائیں گے اور اپنا پہلا multi-node robotic نظام بنائیں گے۔

آگے بڑھنے سے پہلے:
- ✅ تصدیق کریں کہ تمام تنصیبات کام کرتی ہیں
- ✅ [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/) کو bookmark کریں
- ✅ کورس discussion فورم میں شامل ہوں
- ✅ ہفتہ 2 کا quiz مکمل کریں

باب 1 شروع کرنے کے لیے تیار ہیں؟ [ہفتہ 3: ROS 2 Architecture & Core Concepts](../01-ros2/week-03.md) پر جاری رکھیں۔

## اضافی وسائل

- [Ubuntu 22.04 LTS Release Notes](https://wiki.ubuntu.com/JammyJellyfish/ReleaseNotes)
- [Python Virtual Environments Guide](https://docs.python.org/3/tutorial/venv.html)
- [Docker Getting Started](https://docs.docker.com/get-started/)
- [Linux Command Line Cheat Sheet](https://www.linuxtrainingacademy.com/linux-commands-cheat-sheet/)
- [VS Code for Python](https://code.visualstudio.com/docs/python/python-tutorial)

---

## 📝 ہفتہ وار Quiz

اس ہفتے کے مواد کی اپنی سمجھ کو جانچیں! Quiz کثیر انتخابی ہے، خودکار طور پر اسکور کیا جاتا ہے، اور آپ کے پاس 2 کوششیں ہیں۔

**[ہفتہ 2 Quiz لیں →](/quiz?week=2)**
