# Week 2: Development Environment Setup

## Overview

This week focuses on preparing your development environment for the course. You'll install Ubuntu 22.04 (the standard for ROS 2 Humble), set up essential tools, and verify your installation with a simple "Hello World" project. Proper environment setup now will save hours of debugging later!

## Learning Objectives

By the end of this week, you will be able to:

- Install Ubuntu 22.04 LTS (native, dual-boot, WSL2, or VM)
- Configure essential development tools (Python, Git, VS Code)
- Understand Linux basics (terminal, package management, file permissions)
- Install Docker for containerized environments
- Verify your setup with a simple robotics "Hello World"

## Ubuntu 22.04 Installation

ROS 2 Humble (the version used in this course) officially supports **Ubuntu 22.04 LTS (Jammy Jellyfish)**. Choose the installation method that works best for you:

### Option 1: Native Installation (Recommended)

**Best for**: Maximum performance, GPU access, real-time capabilities

**Requirements**: Dedicated machine or dual-boot setup

**Steps**:
1. Download Ubuntu 22.04 Desktop ISO from [ubuntu.com/download](https://ubuntu.com/download/desktop)
2. Create bootable USB with [Rufus](https://rufus.ie/) (Windows) or [Etcher](https://www.balena.io/etcher/) (Mac/Linux)
3. Boot from USB and follow installation wizard
4. Choose "Install alongside Windows" for dual-boot or "Erase disk" for dedicated machine
5. Create user account and set strong password

**Post-install**:
```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Install essential build tools
sudo apt install build-essential git curl wget vim -y
```

### Option 2: WSL2 (Windows Subsystem for Linux)

**Best for**: Windows users who want Linux without dual-boot

**Requirements**: Windows 10 version 2004+ or Windows 11

**Steps**:
```bash
# Run in PowerShell as Administrator
wsl --install -d Ubuntu-22.04

# After installation, launch Ubuntu 22.04 from Start Menu
# Create username and password when prompted

# Inside WSL2, update packages
sudo apt update && sudo apt upgrade -y
```

**GPU Support (for Isaac Sim)**:
- Install [NVIDIA CUDA on WSL2](https://docs.nvidia.com/cuda/wsl-user-guide/index.html)
- Requires NVIDIA driver 510.39.01+ on Windows host

**Limitations**:
- No GUI by default (use X11 forwarding or VcXsrv)
- USB device passthrough is limited
- Slightly slower than native

### Option 3: Virtual Machine (VirtualBox/VMware)

**Best for**: Testing, learning, minimal commitment

**Requirements**: Host machine with 8GB+ RAM, virtualization enabled in BIOS

**Steps** (VirtualBox example):
1. Install [VirtualBox](https://www.virtualbox.org/)
2. Download Ubuntu 22.04 Desktop ISO
3. Create new VM: 4 CPU cores, 8GB RAM, 60GB dynamic disk
4. Mount ISO and install Ubuntu
5. Install VirtualBox Guest Additions for better performance

**Limitations**:
- No GPU passthrough (no NVIDIA Isaac Sim support)
- Limited to Gazebo and lightweight simulations
- Performance overhead

### Option 4: Cloud Instance (AWS/GCP/Azure)

**Best for**: No local hardware, need powerful GPU, temporary use

**Recommended instances**:
- **AWS**: g4dn.xlarge (T4 GPU, $0.526/hr)
- **GCP**: n1-standard-4 + T4 GPU ($0.35/hr + $0.35/hr)
- **Azure**: NC4as_T4_v3 (T4 GPU, $0.526/hr)

**Setup**:
1. Choose Ubuntu 22.04 LTS AMI/image
2. Configure security group (SSH port 22, optionally VNC port 5900)
3. SSH into instance: `ssh -i key.pem ubuntu@<ip-address>`
4. Install desktop environment if needed: `sudo apt install ubuntu-desktop`

**Cost management**:
- Stop instance when not in use
- Use spot/preemptible instances (70% discount)
- Set billing alerts

## Essential Development Tools

### 1. Python 3.11+ Setup

Ubuntu 22.04 ships with Python 3.10. Upgrade to 3.11 for better performance:

```bash
# Add deadsnakes PPA for Python 3.11
sudo add-apt-repository ppa:deadsnakes/ppa -y
sudo apt update

# Install Python 3.11 and tools
sudo apt install python3.11 python3.11-venv python3.11-dev python3-pip -y

# Verify installation
python3.11 --version  # Should show Python 3.11.x

# Set Python 3.11 as default (optional)
sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.11 1

# Install pipenv or poetry for dependency management
pip3 install pipenv poetry
```

### 2. Git Configuration

```bash
# Install Git
sudo apt install git -y

# Configure identity
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"

# Set default branch name
git config --global init.defaultBranch main

# Enable credential caching (avoid repeated password entry)
git config --global credential.helper cache

# Verify configuration
git config --list
```

### 3. VS Code Installation

**Method 1: Snap (Recommended)**
```bash
sudo snap install code --classic
```

**Method 2: .deb Package**
```bash
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" | sudo tee /etc/apt/sources.list.d/vscode.list
sudo apt update
sudo apt install code -y
```

**Recommended Extensions**:
- Python (Microsoft)
- Pylance
- ROS (Microsoft)
- CMake Tools
- Docker
- GitLens

Install via command line:
```bash
code --install-extension ms-python.python
code --install-extension ms-python.vscode-pylance
code --install-extension ms-iot.vscode-ros
code --install-extension ms-vscode.cmake-tools
code --install-extension ms-azuretools.vscode-docker
code --install-extension eamodio.gitlens
```

### 4. Docker Installation

Docker is essential for reproducible environments and will be used in Chapters 3-4.

```bash
# Install dependencies
sudo apt install ca-certificates curl gnupg lsb-release -y

# Add Docker's official GPG key
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

# Set up repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker Engine
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin -y

# Add your user to docker group (avoid sudo for docker commands)
sudo usermod -aG docker $USER

# Log out and back in for group changes to take effect
# Or run: newgrp docker

# Verify installation
docker --version
docker run hello-world
```

### 5. NVIDIA GPU Setup (If Applicable)

For users with NVIDIA GPUs (required for Isaac Sim in Chapter 3):

```bash
# Check GPU
lspci | grep -i nvidia

# Install NVIDIA drivers
sudo apt install nvidia-driver-535 -y  # Or latest stable version
sudo reboot

# Verify driver installation
nvidia-smi  # Should show GPU info

# Install NVIDIA Container Toolkit (for Docker GPU support)
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt update
sudo apt install nvidia-container-toolkit -y

# Configure Docker to use NVIDIA runtime
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Test GPU in Docker
docker run --rm --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi
```

## Linux Command Line Essentials

If you're new to Linux, master these basics:

### Navigation and File Management
```bash
pwd                    # Print working directory
ls -lah                # List files (detailed, including hidden)
cd /path/to/directory  # Change directory
cd ~                   # Go to home directory
cd ..                  # Go up one level

mkdir my_project       # Create directory
touch file.txt         # Create empty file
cp source dest         # Copy file
mv old new             # Move/rename file
rm file.txt            # Delete file
rm -rf directory/      # Delete directory recursively
```

### File Permissions
```bash
chmod +x script.sh     # Make file executable
chmod 644 file.txt     # Set permissions (read/write owner, read others)
chown user:group file  # Change ownership
```

### Package Management
```bash
sudo apt update                  # Update package lists
sudo apt upgrade                 # Upgrade installed packages
sudo apt install <package>       # Install package
sudo apt remove <package>        # Remove package
sudo apt search <keyword>        # Search for packages
```

### Process Management
```bash
ps aux                 # List all processes
top                    # Interactive process monitor
htop                   # Better process monitor (install: sudo apt install htop)
kill <PID>             # Kill process by ID
killall <name>         # Kill processes by name
```

### Text Editing
```bash
nano file.txt          # Simple text editor
vim file.txt           # Advanced editor (learning curve!)
code file.txt          # Open in VS Code
```

## Verification "Hello World" Project

Let's verify your setup with a simple Python project:

### Step 1: Create Project Directory
```bash
mkdir -p ~/robotics_hello_world
cd ~/robotics_hello_world
```

### Step 2: Create Virtual Environment
```bash
python3.11 -m venv venv
source venv/bin/activate  # Activate virtual environment
```

### Step 3: Create Python Script
```bash
code hello_robot.py  # Or use nano/vim
```

Add the following code:
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

    # Simulate simple movements
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
        time.sleep(0.5)  # Simulate real-time delay

    robot.status()
    print("✅ Hello World simulation complete!")

if __name__ == "__main__":
    main()
```

### Step 4: Run the Script
```bash
chmod +x hello_robot.py
python3 hello_robot.py
```

**Expected Output**:
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

✅ Hello World simulation complete!
```

### Step 5: Version Control
```bash
git init
git add hello_robot.py
git commit -m "Initial commit: Hello World robot simulation"
```

## Troubleshooting Common Issues

### Issue 1: "python3.11: command not found"
**Solution**: Python 3.11 not installed. Revisit Python installation section.

### Issue 2: "Permission denied" when running Docker
**Solution**: User not in docker group. Run `sudo usermod -aG docker $USER` and log out/in.

### Issue 3: nvidia-smi shows "NVIDIA-SMI has failed"
**Solution**: Driver not installed or incompatible. Run `sudo apt install nvidia-driver-535` and reboot.

### Issue 4: VS Code extensions not installing
**Solution**: Check internet connection. Try installing manually from extensions marketplace.

### Issue 5: Slow VM performance
**Solution**: Increase RAM/CPU allocation, enable hardware virtualization in BIOS, install guest additions.

## Week 2 Quiz & Assessment

Test your environment setup knowledge:

1. What is the officially supported Ubuntu version for ROS 2 Humble?
2. Why is Ubuntu 22.04 LTS preferred over 24.04 for this course?
3. What is the purpose of a Python virtual environment?
4. How do you check if your NVIDIA GPU is detected in Linux?
5. What is the difference between `apt update` and `apt upgrade`?

**Hands-on Assessment**:
- Run the "Hello World" robot script successfully
- Create a GitHub repository and push your hello_robot.py
- Take a screenshot of `nvidia-smi` output (GPU users only)
- Submit screenshot of VS Code with ROS extension installed

## Next Steps

Congratulations! Your development environment is ready. Next week, you'll dive into **ROS 2 Fundamentals** and build your first multi-node robotic system.

Before proceeding:
- ✅ Verify all installations work
- ✅ Bookmark [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- ✅ Join the course discussion forum
- ✅ Complete the Week 2 quiz

Ready to start Chapter 1? Continue to [Week 3: ROS 2 Architecture & Core Concepts](../01-ros2/week-03.md).

## Additional Resources

- [Ubuntu 22.04 LTS Release Notes](https://wiki.ubuntu.com/JammyJellyfish/ReleaseNotes)
- [Python Virtual Environments Guide](https://docs.python.org/3/tutorial/venv.html)
- [Docker Getting Started](https://docs.docker.com/get-started/)
- [Linux Command Line Cheat Sheet](https://www.linuxtrainingacademy.com/linux-commands-cheat-sheet/)
- [VS Code for Python](https://code.visualstudio.com/docs/python/python-tutorial)
