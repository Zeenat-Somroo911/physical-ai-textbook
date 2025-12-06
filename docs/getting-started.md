---
sidebar_position: 2
---

# Getting Started

This guide will help you set up your development environment for the Physical AI & Humanoid Robotics course. Follow these steps carefully to ensure everything works correctly.

## System Requirements

### Operating System

**Recommended**: Ubuntu 22.04 LTS (Jammy Jellyfish)

- **Why Ubuntu?** ROS 2 officially supports Ubuntu, and most robotics tools are designed for Linux
- **Alternative**: You can use Windows with WSL2 (Windows Subsystem for Linux) or a virtual machine
- **Mac**: Possible but requires additional setup (not officially supported by ROS 2)

### Hardware Requirements

- **CPU**: 64-bit processor (Intel or AMD)
- **RAM**: 8GB minimum, 16GB recommended
- **Storage**: 50GB free disk space
- **Graphics**: Any (GPU optional, not required)
- **Internet**: Required for package downloads

## Installation Methods

### Option 1: Native Ubuntu Installation (Recommended)

If you're on Ubuntu 22.04, follow the steps below.

### Option 2: WSL2 on Windows

1. Install WSL2: `wsl --install`
2. Install Ubuntu 22.04 from Microsoft Store
3. Follow Ubuntu installation steps below

### Option 3: Virtual Machine

1. Install VirtualBox or VMware
2. Download Ubuntu 22.04 ISO
3. Create VM with 8GB RAM, 50GB disk
4. Install Ubuntu in VM
5. Follow Ubuntu installation steps below

## Step 1: Ubuntu 22.04 Setup

### Installing Ubuntu 22.04

If you don't have Ubuntu 22.04:

1. **Download**: Visit [ubuntu.com/download](https://ubuntu.com/download/desktop)
2. **Create Bootable USB**: Use Rufus (Windows) or dd (Linux/Mac)
3. **Install**: Boot from USB and follow installation wizard
4. **Update**: After installation, run:
   ```bash
   sudo apt update && sudo apt upgrade -y
   ```

### Initial System Configuration

```bash
# Set timezone (optional)
sudo timedatectl set-timezone America/New_York

# Install essential tools
sudo apt install -y \
    curl \
    wget \
    git \
    vim \
    build-essential \
    software-properties-common \
    apt-transport-https \
    ca-certificates \
    gnupg \
    lsb-release
```

## Step 2: ROS 2 Humble Installation

### Add ROS 2 Repository

```bash
# Set locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release

# Add ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
```

### Install ROS 2 Humble

```bash
# Update package list
sudo apt update

# Install ROS 2 Humble Desktop (includes RViz, demos, tutorials)
sudo apt install -y ros-humble-desktop

# Install development tools
sudo apt install -y \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

# Initialize rosdep
sudo rosdep init
rosdep update
```

### Source ROS 2 Setup

Add to your `~/.bashrc`:

```bash
# ROS 2 Humble
source /opt/ros/humble/setup.bash
```

Then reload:

```bash
source ~/.bashrc
```

### Verify Installation

```bash
# Check ROS 2 version
ros2 --version

# List available packages
ros2 pkg list | head -20

# Test with a simple example
ros2 run demo_nodes_cpp talker
# In another terminal:
ros2 run demo_nodes_cpp listener
```

## Step 3: Python Setup

### Python Version

ROS 2 Humble requires **Python 3.10**. Ubuntu 22.04 comes with Python 3.10 by default.

```bash
# Check Python version
python3 --version
# Should output: Python 3.10.x

# Install pip if not present
sudo apt install -y python3-pip

# Upgrade pip
python3 -m pip install --upgrade pip
```

### Python Packages

```bash
# Install essential packages
pip3 install --user \
    numpy \
    matplotlib \
    opencv-python \
    pyyaml \
    setuptools \
    wheel

# For Module 3 (Computer Vision)
pip3 install --user \
    ultralytics \
    opencv-contrib-python

# For Module 4 (VLA)
pip3 install --user \
    openai \
    speechrecognition \
    pyaudio
```

### Create Workspace

```bash
# Create ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build workspace
colcon build

# Source workspace
source install/setup.bash

# Add to ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Step 4: VS Code Configuration

### Install VS Code

```bash
# Download and install VS Code
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/trusted.gpg.d/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'

sudo apt update
sudo apt install -y code
```

### VS Code Extensions

Install these extensions in VS Code:

1. **ROS** (ms-ros) - ROS 2 support
2. **Python** (ms-python) - Python language support
3. **C/C++** (ms-vscode) - C++ support (optional)
4. **YAML** (redhat) - YAML file support
5. **Markdown All in One** (yzhang) - Markdown support

### VS Code Settings

Create `~/.vscode/settings.json`:

```json
{
    "python.defaultInterpreterPath": "/usr/bin/python3",
    "python.linting.enabled": true,
    "python.linting.pylintEnabled": true,
    "files.associations": {
        "*.launch.py": "python",
        "*.urdf": "xml",
        "*.sdf": "xml"
    },
    "editor.formatOnSave": true,
    "editor.tabSize": 2
}
```

## Step 5: Additional Tools

### Gazebo Installation

```bash
# Install Gazebo (for Module 2)
sudo apt install -y \
    gazebo11 \
    libgazebo11-dev \
    ros-humble-gazebo-ros-pkgs
```

### Git Configuration

```bash
# Configure Git
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"

# Set default branch name
git config --global init.defaultBranch main
```

### Useful Aliases

Add to `~/.bashrc`:

```bash
# ROS 2 aliases
alias ros2ws='cd ~/ros2_ws'
alias ros2src='cd ~/ros2_ws/src'
alias ros2build='cd ~/ros2_ws && colcon build'
alias ros2source='source ~/ros2_ws/install/setup.bash'
```

## Step 6: Verify Complete Setup

### Test Script

Create `~/test_setup.sh`:

```bash
#!/bin/bash

echo "=== Testing ROS 2 Setup ==="

# Check ROS 2
if command -v ros2 &> /dev/null; then
    echo "✅ ROS 2 installed: $(ros2 --version)"
else
    echo "❌ ROS 2 not found"
fi

# Check Python
if command -v python3 &> /dev/null; then
    echo "✅ Python installed: $(python3 --version)"
else
    echo "❌ Python not found"
fi

# Check workspace
if [ -d ~/ros2_ws ]; then
    echo "✅ ROS 2 workspace exists"
else
    echo "❌ ROS 2 workspace not found"
fi

# Check packages
python3 -c "import numpy; print('✅ NumPy installed')" 2>/dev/null || echo "❌ NumPy not found"
python3 -c "import cv2; print('✅ OpenCV installed')" 2>/dev/null || echo "❌ OpenCV not found"

echo "=== Setup Test Complete ==="
```

Run it:

```bash
chmod +x ~/test_setup.sh
~/test_setup.sh
```

## Troubleshooting

### Issue 1: ROS 2 Command Not Found

**Error**: `ros2: command not found`

**Solution**:
```bash
# Source ROS 2 setup
source /opt/ros/humble/setup.bash

# Add to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Issue 2: Permission Denied

**Error**: `Permission denied` when running commands

**Solution**:
```bash
# Don't use sudo for ROS 2 commands
# Only use sudo for apt install

# If you get permission errors with pip, use --user flag
pip3 install --user package_name
```

### Issue 3: Python Import Errors

**Error**: `ModuleNotFoundError: No module named 'cv2'`

**Solution**:
```bash
# Install missing package
pip3 install --user opencv-python

# If using virtual environment, activate it first
python3 -m venv venv
source venv/bin/activate
pip install opencv-python
```

### Issue 4: Gazebo Won't Start

**Error**: Gazebo crashes or won't launch

**Solution**:
```bash
# Check graphics drivers
glxinfo | grep "OpenGL version"

# Update graphics drivers if needed
sudo ubuntu-drivers autoinstall

# Try software rendering
export LIBGL_ALWAYS_SOFTWARE=1
gazebo
```

### Issue 5: Workspace Build Fails

**Error**: `colcon build` fails

**Solution**:
```bash
# Install build dependencies
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep

# Install package dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Clean and rebuild
rm -rf build install log
colcon build
```

### Issue 6: VS Code Can't Find ROS 2

**Error**: VS Code ROS extension can't find ROS 2

**Solution**:
1. Open VS Code
2. Press `Ctrl+Shift+P`
3. Type "ROS: Set ROS Distribution"
4. Select "humble"
5. Restart VS Code

## Next Steps

Once your environment is set up:

1. **Verify Installation**: Run the test script above
2. **Start Learning**: Begin with [Module 1: ROS 2 Fundamentals](../module-01-ros2/01-introduction)
3. **Build First Node**: Create your first ROS 2 node
4. **Join Community**: Connect with other learners

## Additional Resources

- **ROS 2 Documentation**: [docs.ros.org](https://docs.ros.org/en/humble/)
- **Gazebo Tutorials**: [classic.gazebosim.org/tutorials](http://classic.gazebosim.org/tutorials)
- **Python Tutorial**: [docs.python.org/3/tutorial](https://docs.python.org/3/tutorial/)
- **VS Code Docs**: [code.visualstudio.com/docs](https://code.visualstudio.com/docs)

## Getting Help

If you encounter issues:

1. Check the [FAQ](faq.md) for common problems
2. Search [ROS Discourse](https://discourse.ros.org)
3. Ask in our community forums
4. Review error messages carefully

---

**Ready to start building robots?** Head to [Module 1](../module-01-ros2/01-introduction)! 🤖

