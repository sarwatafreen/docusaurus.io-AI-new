---
sidebar_position: 7
---

# Docker Setup: One-Click Start

This guide provides instructions for setting up and running the complete Physical AI & Humanoid Robotics environment using Docker. This approach allows you to run everything without installing dependencies on your local machine.

## Quick Start (One-Click)

The fastest way to get started is using our pre-built Docker setup:

```bash
# Clone the repository
git clone https://github.com/physical-ai-book/physical-ai-environment.git
cd physical-ai-environment

# Run the one-click setup
./start.sh
```

Or if you prefer to build yourself:

```bash
# Clone and build
git clone https://github.com/physical-ai-book/physical-ai-environment.git
cd physical-ai-environment

# Build the Docker image
docker-compose build

# Start all services
docker-compose up -d
```

## Docker Configuration

### Dockerfile

Our Dockerfile includes all necessary components for the course:

```dockerfile
# Use ROS 2 Humble base image
FROM osrf/ros:humble-desktop

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    curl \
    wget \
    vim \
    nano \
    gnupg \
    lsb-release \
    build-essential \
    cmake \
    pkg-config \
    libusb-1.0-0-dev \
    libtbb-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libgtk-3-dev \
    libatlas-base-dev \
    gfortran \
    libhdf5-dev \
    libhdf5-serial-dev \
    libhdf5-103 \
    libqtgui4 \
    libqtwebkit4 \
    libqt4-test \
    python3-pyqt5 \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR /workspace
RUN mkdir -p /workspace/src

# Install Python dependencies
RUN pip3 install --upgrade pip
COPY requirements.txt /workspace/requirements.txt
RUN pip3 install -r requirements.txt

# Install Isaac Sim dependencies if available
# (Limited in Docker due to GPU requirements)

# Install OpenCV
RUN pip3 install opencv-python

# Copy project files
COPY . /workspace/

# Source ROS environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc

# Set up entrypoint
COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
```

### requirements.txt

Key Python dependencies for the course:

```
openai==0.28.1
speechrecognition==3.10.0
pyaudio==0.2.11
transformers==4.35.0
torch==2.1.0
torchvision==0.16.0
torchaudio==2.1.0
numpy==1.24.3
opencv-python==4.8.1.78
ros2-launch==1.0.0
setuptools==68.0.0
```

### docker-compose.yml

Full Docker Compose configuration:

```yaml
version: '3.8'

services:
  physical-ai-book:
    build: .
    container_name: physical-ai-book
    ports:
      - "3000:3000"    # Docusaurus dev server
      - "8000:8000"    # Additional web app ports
      - "11311:11311"  # ROS master
      - "8080:8080"    # Web tools
      - "11345:11345"  # Additional ROS services
    volumes:
      - .:/workspace
      - ~/.ssh:/root/.ssh:ro  # For git access
    environment:
      - DISPLAY=${DISPLAY}
      - ROS_DOMAIN_ID=0
      - RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
    network_mode: host  # For ROS communication
    stdin_open: true
    tty: true
    privileged: true
    
  # Optional: Jupyter notebook for interactive development
  jupyter:
    image: jupyter/scipy-notebook:latest
    container_name: physical-ai-jupyter
    ports:
      - "8888:8888"
    volumes:
      - .:/workspace
    environment:
      - JUPYTER_ENABLE_LAB=yes
    command: start-notebook.sh --NotebookApp.token='' --NotebookApp.password=''

  # Optional: Visualization tools
  rviz:
    image: osrf/ros:humble-desktop
    container_name: physical-ai-rviz
    ports:
      - "11312:11311"
    environment:
      - DISPLAY=${DISPLAY}
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
    network_mode: host
    privileged: true
    command: >
      bash -c "
      apt-get update &&
      apt-get install -y ros-humble-rviz2 &&
      source /opt/ros/humble/setup.bash &&
      ros2 run rviz2 rviz2
      "
```

## Running the Environment

### Basic Usage

1. **Start the environment:**
```bash
# Build and start all services
docker-compose up --build

# Or start in background
docker-compose up -d
```

2. **Access the services:**
- Documentation: http://localhost:3000
- Jupyter Notebook: http://localhost:8888
- ROS tools: Available in the container terminal

3. **Enter the container:**
```bash
# Access the main container
docker exec -it physical-ai-book bash

# Access with ROS environment pre-sourced
docker exec -it physical-ai-book bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && bash"
```

### Development Workflow in Docker

```bash
# Enter the container
docker exec -it physical-ai-book bash

# Source ROS environment
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

# Build your ROS packages
cd /workspace
colcon build

# Run ROS nodes
ros2 run your_package your_node

# Launch systems
ros2 launch your_package your_launch.py
```

## Performance Optimization

### Resource Allocation

For better performance, allocate sufficient resources to Docker:

- **Memory**: At least 8GB (16GB recommended for Isaac Sim simulation)
- **CPU**: 4+ cores for optimal performance
- **Disk**: 50GB+ free space

In Docker Desktop settings:
```bash
# Windows/Mac: Settings → Resources → Advanced
# Set Memory to 12GB+
# Set CPUs to 6+
```

### GPU Acceleration (Optional)

If you have an NVIDIA GPU and need acceleration:

1. Install NVIDIA Container Toolkit
2. Run with GPU flag:
```bash
# For Isaac Sim or other GPU-accelerated tasks
docker run --gpus all -it your-image:tag
```

### Docker Compose with GPU

```yaml
version: '3.8'

services:
  physical-ai-book-gpu:
    build: .
    container_name: physical-ai-book-gpu
    ports:
      - "3000:3000"
      - "8000:8000"
      - "11311:11311"
    volumes:
      - .:/workspace
    environment:
      - DISPLAY=${DISPLAY}
      - NVIDIA_VISIBLE_DEVICES=all
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]
    stdin_open: true
    tty: true
```

## Troubleshooting

### Common Issues

**Problem:** Container fails to start with GUI applications
**Solution:** Set up X11 forwarding (Linux) or install X server (Windows/Mac):
```bash
# Linux
xhost +local:docker
docker run --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" your-image

# Windows/Mac: Install X410 or XQuartz respectively
```

**Problem:** ROS nodes can't communicate between containers
**Solution:** Use host networking or configure ROS_DOMAIN_ID consistently:
```bash
export ROS_DOMAIN_ID=0
```

**Problem:** Isaac Sim doesn't run in Docker
**Solution:** Isaac Sim requires significant GPU resources not fully compatible with Docker. Use native installation for Isaac Sim.

**Problem:** Audio doesn't work in container
**Solution:** Pass audio devices to container:
```bash
# Linux
docker run --device=/dev/snd your-image

# Or use pulseaudio
docker run -v /run/user/$(id -u)/pulse:/run/user/$(id -u)/pulse -e PULSE_SERVER=unix:/run/user/$(id -u)/pulse/native your-image
```

## Quiz

1. What are the benefits of using Docker for the Physical AI course?
2. How do you access the documentation within the Docker setup?
3. What should you do if you encounter GUI issues with Docker?

## Summary

This Docker setup provides a complete, isolated environment for the Physical AI course. It includes all necessary dependencies and tools, allowing you to focus on learning robotics without worrying about environment setup. The one-click setup makes it easy to get started, while the configurable options allow advanced users to optimize for their specific needs.

Continue to the next sections to start learning robotics with your ready-to-use development environment!