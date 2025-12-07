---
sidebar_position: 6
---

# Hardware & Setup Guide

This guide provides comprehensive information for setting up your physical AI development environment, whether you're using the recommended hardware kit or the free Docker alternative.

## Recommended Hardware Kit (~$700 total)

### NVIDIA Jetson Orin Nano 8GB - $249
The Jetson Orin Nano provides powerful AI compute capabilities at an affordable price point, making it ideal for robotics development.

**Why it's essential:**
- 100+ TOPS AI performance
- ARM64 architecture (matches many robot platforms)
- Power efficiency optimized for mobile robots
- NVIDIA CUDA support for AI acceleration

**Setup:**
1. Connect power adapter (official 15W+ recommended)
2. Connect HDMI to monitor
3. Connect USB keyboard and mouse
4. Insert microSD card with JetPack SDK
5. Follow on-screen instructions for initial setup

### Intel RealSense D435i Camera - $349
A stereo camera with built-in IMU and depth sensing capabilities, perfect for robotics perception.

**Key features:**
- RGB camera: 1920x1080 @ 30fps
- Depth sensing: up to 10m range
- Built-in IMU for motion tracking
- ROS 2 drivers available

**Mounting:**
- Secure to robot's head or front panel
- Connect via USB 3.0 for full capabilities
- Calibrate using provided tools

### USB Microphone (ReSpeaker 4-Mic Array) - $69
High-quality audio capture designed for voice recognition applications.

**Features:**
- Far-field voice capture
- Acoustic echo cancellation
- 4-microphone array for beamforming
- USB connection with minimal latency

## Alternative: Docker Setup (Free)

If you don't have the hardware kit, you can run everything in Docker. Note that performance will be slower than native hardware.

### Prerequisites
- Docker installed (version 20.10+)
- At least 8GB RAM (16GB recommended)
- 50GB+ free disk space

### Setup Steps

1. **Install Docker:**
```bash
# For Ubuntu/Debian
sudo apt update
sudo apt install docker.io
sudo usermod -aG docker $USER  # Add yourself to docker group
newgrp docker  # Activate group changes

# For Windows/Mac, download Docker Desktop from docker.com
```

2. **Clone the Project Repository:**
```bash
git clone https://github.com/your-username/physical-ai-book.git
cd physical-ai-book
```

3. **Build the Docker Image:**
```bash
# Build the image with all dependencies
docker build -t physical-ai-book -f Dockerfile .

# Or pull from registry if available
docker pull your-repo/physical-ai-book:latest
```

4. **Run the Development Environment:**
```bash
# For Linux with GPU support
docker run -it --gpus all \
  --name physical-ai-dev \
  -v $(pwd):/workspace \
  -p 8000:8000 \
  -p 5901:5901 \
  --privileged \
  physical-ai-book:latest

# For Windows/Mac without GPU
docker run -it \
  --name physical-ai-dev \
  -v ${PWD}:/workspace \
  -p 8000:8000 \
  physical-ai-book:latest
```

### Docker Compose Setup (Alternative)

For easier management, use docker-compose:

```yaml
# docker-compose.yml
version: '3.8'
services:
  physical-ai-book:
    build: .
    container_name: physical-ai-book
    ports:
      - "8000:8000"  # Docusaurus dev server
      - "11311:11311"  # ROS master
      - "8080:8080"  # Web tools
    volumes:
      - .:/workspace
      - /tmp/.X11-unix:/tmp/.X11-unix:rw  # GUI support (Linux)
    environment:
      - DISPLAY=${DISPLAY}
      - NVIDIA_VISIBLE_DEVICES=all
    runtime: nvidia  # For GPU support
    privileged: true
    command: >
      bash -c "
      cd /workspace &&
      source /opt/ros/humble/setup.bash &&
      source install/setup.bash &&
      bash"
```

Run with:
```bash
docker-compose up -d
```

## Software Setup

### ROS 2 Humble Hawksbill
Install ROS 2 Humble Hawksbill (long-term support version):

```bash
# For Ubuntu 22.04
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-pip
sudo rosdep init
rosdep update
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Isaac Sim Setup
For those with compatible NVIDIA hardware:

```bash
# Download Isaac Sim from NVIDIA Developer website
# Follow the installation guide:
# 1. Install Omniverse Launcher
# 2. Install Isaac Sim through launcher
# 3. Configure for ROS 2 integration
```

### Additional Dependencies

Install required packages for the course:

```bash
pip3 install openai speechrecognition pyaudio transformers torch
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-isaac-sim-bridge  # If available
```

## Troubleshooting

### Common Issues

**Problem:** Docker container fails to start with GPU support
**Solution:** Verify NVIDIA drivers and Docker NVIDIA runtime:
```bash
nvidia-smi  # Should show your GPU
docker run --gpus all nvidia/cuda:11.0-base nvidia-smi  # Test GPU access
```

**Problem:** Isaac Sim fails to launch
**Solution:** Ensure you have a compatible NVIDIA GPU with recent drivers:
```bash
nvidia-smi  # Must show RTX series or better
# Update drivers if needed: sudo apt install nvidia-driver-470
```

**Problem:** Voice recognition fails with "device not found"
**Solution:** Check microphone permissions and configuration:
```bash
arecord -l  # List audio devices
# In Docker, ensure audio devices are passed through
```

### Performance Tips

- For Isaac Sim, ensure you have a modern RTX card (3060 or better)
- Allocate sufficient RAM to Docker (8GB+ for Isaac Sim)
- Use SSD storage for better simulation performance
- Close unnecessary applications during intensive simulation

## Quiz

1. What are the three main components of the recommended hardware kit?
2. Why is the Jetson Orin Nano specifically recommended for robotics?
3. How much does the complete recommended kit cost?

## Summary

This hardware and setup guide provides all the information you need to get started with the Physical AI course, whether you're using the recommended hardware kit or the free Docker alternative. Choose the option that best fits your budget and capabilities, and ensure you can successfully set up your development environment before proceeding to Module 1.