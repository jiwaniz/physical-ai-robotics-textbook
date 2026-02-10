# ہفتہ 2: ڈیولپمنٹ ماحول سیٹ اپ

## جائزہ

یہ ہفتہ کورس کے لیے آپ کے ڈیولپمنٹ ماحول کو تیار کرنے پر مرکوز ہے۔ آپ اوبنٹو 22.04 انسٹال کریں گے (آر او ایس 2 ہمبل کے لیے معیار)، ضروری ٹولز سیٹ اپ کریں گے، اور ایک سادہ "ہیلو ورلڈ" پروجیکٹ کے ساتھ اپنی تنصیب کی تصدیق کریں گے۔ مناسب ماحول کا سیٹ اپ اب بعد میں ڈیبگنگ کے گھنٹے بچائے گا!

## سیکھنے کے مقاصد

اس ہفتے کے اختتام تک، آپ قابل ہوں گے:

- اوبنٹو 22.04 ایل ٹی ایس انسٹال کریں (نیٹو، ڈوئل بوٹ، ڈبلیو ایس ایل 2، یا وی ایم)
- ضروری ڈیولپمنٹ ٹولز کنفیگر کریں (پائتھون، گٹ، وی ایس کوڈ)
- لینکس کی بنیادی باتیں سمجھیں (ٹرمینل، پیکیج مینجمنٹ، فائل پرمیشنز)
- کنٹینرائزڈ ماحول کے لیے ڈاکر انسٹال کریں
- سادہ روبوٹکس "ہیلو ورلڈ" کے ساتھ اپنے سیٹ اپ کی تصدیق کریں

## اوبنٹو 22.04 تنصیب

آر او ایس 2 ہمبل (اس کورس میں استعمال شدہ ورژن) سرکاری طور پر **اوبنٹو 22.04 ایل ٹی ایس (جیمی جیلی فش)** کو سپورٹ کرتا ہے۔ وہ تنصیب کا طریقہ منتخب کریں جو آپ کے لیے بہترین کام کرے:

### آپشن 1: نیٹو تنصیب (تجویز کردہ)

**بہترین برائے**: زیادہ سے زیادہ کارکردگی، جی پی یو رسائی، ریئل ٹائم صلاحیتیں

**ضروریات**: مخصوص مشین یا ڈوئل بوٹ سیٹ اپ

**اقدامات**:
1. [ubuntu.com/download](https://ubuntu.com/download/desktop) سے اوبنٹو 22.04 ڈیسک ٹاپ آئی ایس او ڈاؤن لوڈ کریں
2. [Rufus](https://rufus.ie/) (ونڈوز) یا [Etcher](https://www.balena.io/etcher/) (میک/لینکس) کے ساتھ بوٹ ایبل یو ایس بی بنائیں
3. یو ایس بی سے بوٹ کریں اور انسٹالیشن وزرڈ کی پیروی کریں
4. ڈوئل بوٹ کے لیے "ونڈوز کے ساتھ انسٹال کریں" یا مخصوص مشین کے لیے "ڈسک صاف کریں" منتخب کریں
5. صارف کا اکاؤنٹ بنائیں اور مضبوط پاس ورڈ سیٹ کریں

**پوسٹ-انسٹال**:
```bash
# سسٹم پیکجز کو اپ ڈیٹ کریں
sudo apt update && sudo apt upgrade -y

# ضروری build ٹولز انسٹال کریں
sudo apt install build-essential git curl wget vim -y
```

### آپشن 2: ڈبلیو ایس ایل 2 (ونڈوز سب سسٹم فار لینکس)

**بہترین برائے**: ونڈوز صارفین جو ڈوئل بوٹ کے بغیر لینکس چاہتے ہیں

**ضروریات**: ونڈوز 10 ورژن 2004+ یا ونڈوز 11

**اقدامات**:
```bash
# PowerShell میں Administrator کے طور پر چلائیں
wsl --install -d Ubuntu-22.04

# تنصیب کے بعد، Start Menu سے Ubuntu 22.04 شروع کریں
# پوچھے جانے پر username اور password بنائیں

# WSL2 کے اندر، پیکجز کو اپ ڈیٹ کریں
sudo apt update && sudo apt upgrade -y
```

**جی پی یو سپورٹ (آئزک سم کے لیے)**:
- [این ویڈیا کوڈا آن ڈبلیو ایس ایل 2](https://docs.nvidia.com/cuda/wsl-user-guide/index.html) انسٹال کریں
- ونڈوز ہوسٹ پر این ویڈیا ڈرائیور 510.39.01+ کی ضرورت ہے

**حدود**:
- ڈیفالٹ طور پر کوئی جی یو آئی نہیں (ایکس11 فارورڈنگ یا وی سی ایکس ایس آر وی استعمال کریں)
- یو ایس بی ڈیوائس پاس تھرو محدود ہے
- نیٹو سے تھوڑا سست

### آپشن 3: ورچوئل مشین (ورچوئل باکس/وی ایم ویئر)

**بہترین برائے**: ٹیسٹنگ، سیکھنا، کم وابستگی

**ضروریات**: 8 جی بی+ ریم والی ہوسٹ مشین، بائیوس میں ورچوئلائزیشن فعال

**اقدامات** (ورچوئل باکس مثال):
1. [ورچوئل باکس](https://www.virtualbox.org/) انسٹال کریں
2. اوبنٹو 22.04 ڈیسک ٹاپ آئی ایس او ڈاؤن لوڈ کریں
3. نیا وی ایم بنائیں: 4 سی پی یو کورز، 8 جی بی ریم، 60 جی بی ڈائنامک ڈسک
4. آئی ایس او ماؤنٹ کریں اور اوبنٹو انسٹال کریں
5. بہتر کارکردگی کے لیے ورچوئل باکس گیسٹ ایڈیشنز انسٹال کریں

**حدود**:
- کوئی جی پی یو پاس تھرو نہیں (این ویڈیا آئزک سم سپورٹ نہیں)
- گزیبو اور ہلکے سمیولیشنز تک محدود
- کارکردگی کا اوور ہیڈ

### آپشن 4: کلاؤڈ انسٹینس (اے ڈبلیو ایس/جی سی پی/ایژر)

**بہترین برائے**: کوئی مقامی ہارڈویئر نہیں، طاقتور جی پی یو کی ضرورت، عارضی استعمال

**تجویز کردہ انسٹینسز**:
- **اے ڈبلیو ایس**: جی4ڈی این.ایکس لارج (ٹی4 جی پی یو، $0.526/گھنٹہ)
- **جی سی پی**: این1-سٹینڈرڈ-4 + ٹی4 جی پی یو ($0.35/گھنٹہ + $0.35/گھنٹہ)
- **ایژر**: این سی 4 اے ایس_ٹی4_وی3 (ٹی4 جی پی یو، $0.526/گھنٹہ)

**سیٹ اپ**:
1. اوبنٹو 22.04 ایل ٹی ایس اے ایم آئی/امیج منتخب کریں
2. سیکیورٹی گروپ کنفیگر کریں (ایس ایس ایچ پورٹ 22، اختیاری طور پر وی این سی پورٹ 5900)
3. انسٹینس میں ایس ایس ایچ کریں: `ssh -i key.pem ubuntu@<ip-address>`
4. اگر ضرورت ہو تو ڈیسک ٹاپ ماحول انسٹال کریں: `sudo apt install ubuntu-desktop`

**لاگت کا انتظام**:
- استعمال میں نہ ہونے پر انسٹینس بند کریں
- اسپاٹ/پری ایمپٹیبل انسٹینسز استعمال کریں (70% رعایت)
- بلنگ الرٹس سیٹ کریں

## ضروری ڈیولپمنٹ ٹولز

### 1. پائتھون 3.11+ سیٹ اپ

اوبنٹو 22.04 پائتھون 3.10 کے ساتھ آتا ہے۔ بہتر کارکردگی کے لیے 3.11 میں اپ گریڈ کریں:

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

### 2. گٹ کنفیگریشن

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

### 3. وی ایس کوڈ تنصیب

**طریقہ 1: سنیپ (تجویز کردہ)**
```bash
sudo snap install code --classic
```

**طریقہ 2: .deb پیکیج**
```bash
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" | sudo tee /etc/apt/sources.list.d/vscode.list
sudo apt update
sudo apt install code -y
```

**تجویز کردہ ایکسٹینشنز**:
- پائتھون (مائیکروسافٹ)
- پائلینس
- آر او ایس (مائیکروسافٹ)
- سی میک ٹولز
- ڈاکر
- گٹ لینز

کمانڈ لائن کے ذریعے انسٹال کریں:
```bash
code --install-extension ms-python.python
code --install-extension ms-python.vscode-pylance
code --install-extension ms-iot.vscode-ros
code --install-extension ms-vscode.cmake-tools
code --install-extension ms-azuretools.vscode-docker
code --install-extension eamodio.gitlens
```

### 4. ڈاکر تنصیب

ڈاکر قابل تکرار ماحول کے لیے ضروری ہے اور باب 3-4 میں استعمال کیا جائے گا۔

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

### 5. این ویڈیا جی پی یو سیٹ اپ (اگر قابل اطلاق ہو)

این ویڈیا جی پی یوز والے صارفین کے لیے (باب 3 میں آئزک سم کے لیے ضروری):

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

## لینکس کمانڈ لائن ضروری باتیں

اگر آپ لینکس میں نئے ہیں، تو یہ بنیادی باتیں سیکھیں:

### نیویگیشن اور فائل مینجمنٹ
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

### فائل پرمیشنز
```bash
chmod +x script.sh     # فائل کو executable بنائیں
chmod 644 file.txt     # Permissions سیٹ کریں (owner read/write، دیگر read)
chown user:group file  # Ownership تبدیل کریں
```

### پیکیج مینجمنٹ
```bash
sudo apt update                  # Package lists کو اپ ڈیٹ کریں
sudo apt upgrade                 # انسٹال شدہ packages کو اپ گریڈ کریں
sudo apt install <package>       # Package انسٹال کریں
sudo apt remove <package>        # Package ہٹائیں
sudo apt search <keyword>        # Packages تلاش کریں
```

### پروسیس مینجمنٹ
```bash
ps aux                 # تمام processes کی فہرست
top                    # Interactive process monitor
htop                   # بہتر process monitor (انسٹال: sudo apt install htop)
kill <PID>             # ID کے ذریعے process کو kill کریں
killall <name>         # نام کے ذریعے processes کو kill کریں
```

### ٹیکسٹ ایڈیٹنگ
```bash
nano file.txt          # سادہ text editor
vim file.txt           # جدید editor (سیکھنے کا curve!)
code file.txt          # VS Code میں کھولیں
```

## تصدیقی "ہیلو ورلڈ" پروجیکٹ

آئیے ایک سادہ پائتھون پروجیکٹ کے ساتھ اپنے سیٹ اپ کی تصدیق کریں:

### قدم 1: پروجیکٹ ڈائریکٹری بنائیں
```bash
mkdir -p ~/robotics_hello_world
cd ~/robotics_hello_world
```

### قدم 2: ورچوئل ماحول بنائیں
```bash
python3.11 -m venv venv
source venv/bin/activate  # Virtual environment کو فعال کریں
```

### قدم 3: پائتھون اسکرپٹ بنائیں
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

### قدم 4: اسکرپٹ چلائیں
```bash
chmod +x hello_robot.py
python3 hello_robot.py
```

**متوقع آؤٹ پٹ**:
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

### قدم 5: ورژن کنٹرول
```bash
git init
git add hello_robot.py
git commit -m "Initial commit: Hello World robot simulation"
```

## عام مسائل کا حل

### مسئلہ 1: "python3.11: command not found"
**حل**: پائتھون 3.11 انسٹال نہیں ہے۔ پائتھون تنصیب کے حصے پر دوبارہ جائیں۔

### مسئلہ 2: ڈاکر چلاتے وقت "Permission denied"
**حل**: صارف ڈاکر گروپ میں نہیں ہے۔ `sudo usermod -aG docker $USER` چلائیں اور لاگ آؤٹ/ان کریں۔

### مسئلہ 3: `nvidia-smi` "NVIDIA-SMI has failed" دکھاتا ہے
**حل**: ڈرائیور انسٹال نہیں ہے یا غیر موافق ہے۔ `sudo apt install nvidia-driver-535` چلائیں اور ری بوٹ کریں۔

### مسئلہ 4: وی ایس کوڈ ایکسٹینشنز انسٹال نہیں ہو رہے
**حل**: انٹرنیٹ کنکشن چیک کریں۔ ایکسٹینشنز مارکیٹ پلیس سے دستی طور پر انسٹال کرنے کی کوشش کریں۔

### مسئلہ 5: وی ایم کی سست کارکردگی
**حل**: ریم/سی پی یو ایلوکیشن بڑھائیں، بائیوس میں ہارڈویئر ورچوئلائزیشن فعال کریں، گیسٹ ایڈیشنز انسٹال کریں۔

## ہفتہ 2 کوئز اور تشخیص

اپنے ماحول کے سیٹ اپ کے علم کو جانچیں:

1. آر او ایس 2 ہمبل کے لیے سرکاری طور پر سپورٹڈ اوبنٹو ورژن کیا ہے؟
2. اس کورس کے لیے اوبنٹو 24.04 کے مقابلے میں اوبنٹو 22.04 ایل ٹی ایس کیوں ترجیح دی جاتی ہے؟
3. پائتھون ورچوئل ماحول کا مقصد کیا ہے؟
4. آپ لینکس میں یہ کیسے چیک کرتے ہیں کہ آیا آپ کا این ویڈیا جی پی یو ڈٹیکٹ ہو رہا ہے؟
5. `apt update` اور `apt upgrade` میں کیا فرق ہے؟

**ہاتھوں سے تشخیص**:
- "ہیلو ورلڈ" روبوٹ اسکرپٹ کامیابی سے چلائیں
- گٹ ہب ریپوزٹری بنائیں اور اپنا hello_robot.py پش کریں
- `nvidia-smi` آؤٹ پٹ کا اسکرین شاٹ لیں (صرف جی پی یو صارفین)
- آر او ایس ایکسٹینشن کے ساتھ انسٹال شدہ وی ایس کوڈ کا اسکرین شاٹ جمع کرائیں

## اگلے اقدامات

مبارک ہو! آپ کا ڈیولپمنٹ ماحول تیار ہے۔ اگلے ہفتے، آپ **آر او ایس 2 بنیادی باتوں** میں غوطہ لگائیں گے اور اپنا پہلا ملٹی-نوڈ روبوٹک نظام بنائیں گے۔

آگے بڑھنے سے پہلے:
- ✅ تصدیق کریں کہ تمام تنصیبات کام کرتی ہیں
- ✅ [آر او ایس 2 ہمبل ڈاکومینٹیشن](https://docs.ros.org/en/humble/) کو بک مارک کریں
- ✅ کورس ڈسکشن فورم میں شامل ہوں
- ✅ ہفتہ 2 کا کوئز مکمل کریں

باب 1 شروع کرنے کے لیے تیار ہیں؟ [ہفتہ 3: آر او ایس 2 آرکیٹیکچر اور بنیادی تصورات](../01-ros2/week-03.md) پر جاری رکھیں۔

## اضافی وسائل

- [اوبنٹو 22.04 ایل ٹی ایس ریلیز نوٹس](https://wiki.ubuntu.com/JammyJellyfish/ReleaseNotes)
- [پائتھون ورچوئل ماحول گائیڈ](https://docs.python.org/3/tutorial/venv.html)
- [ڈاکر شروع کرنا](https://docs.docker.com/get-started/)
- [لینکس کمانڈ لائن چیٹ شیٹ](https://www.linuxtrainingacademy.com/linux-commands-cheat-sheet/)
- [وی ایس کوڈ فار پائتھون](https://code.visualstudio.com/docs/python/python-tutorial)

---

## 📝 ہفتہ وار کوئز

اس ہفتے کے مواد کی اپنی سمجھ کو جانچیں! کوئز کثیر انتخابی ہے، خودکار طور پر اسکور کیا جاتا ہے، اور آپ کے پاس 2 کوششیں ہیں۔

**[ہفتہ 2 کوئز لیں →](/quiz?week=2)**
