---
sidebar_position: 8
---

# Troubleshooting Guide

This guide provides solutions for common issues you might encounter during the Physical AI course. If you encounter an issue not listed here, please check the course community forums.

## ROS 2 Common Issues

### ROS 2 Installation Issues

**Problem:** `rosdep update` fails with timeout or network errors
**Solution:** 
```bash
# Retry with increased timeout
rosdep update --rosdistro humble --os=ubuntu:jammy

# Or try with different mirrors
export ROSDISTRO_INDEX_URL="https://raw.githubusercontent.com/ros/rosdistro/master/index-v4.yaml"
rosdep update
```

### ROS 2 Workspace Issues

**Problem:** `colcon build` fails with missing dependencies
**Solution:**
```bash
# Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# Clean build and retry
rm -rf build/ install/ log/
colcon build
```

**Problem:** Packages not found after successful build
**Solution:**
```bash
# Source the setup file
source install/setup.bash

# Or add to your bashrc
echo "source /path/to/your/workspace/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Isaac Sim Issues

### Installation Problems

**Problem:** Isaac Sim fails to launch
**Solutions:**
1. Ensure you have an RTX-class NVIDIA GPU
2. Update your NVIDIA drivers to the latest version
3. Check that Omniverse is properly installed

**Problem:** Isaac Sim runs but with poor performance
**Solutions:**
1. Reduce the rendering quality in settings
2. Close other GPU-intensive applications
3. Ensure you have sufficient RAM (16GB+ recommended)

### Simulation Issues

**Problem:** Robot doesn't respond in Isaac Sim
**Solution:**
```bash
# Check if ROS bridge is running
ros2 topic list | grep cmd_vel

# Verify the robot's controller is active
ros2 run rqt_graph rqt_graph
```

## Audio/Voice Recognition Issues

### Microphone Problems

**Problem:** Voice recognition doesn't detect speech
**Solutions:**
1. Check microphone permissions in Docker:
```bash
# Run Docker with audio access
docker run --device=/dev/snd your-image
```

2. Test audio with:
```bash
# Check available audio devices
arecord -l

# Test recording
arecord -D hw:0,0 -f cd test.wav
aplay test.wav
```

### Whisper API Issues

**Problem:** OpenAI API returns errors
**Solutions:**
1. Verify your API key:
```bash
export OPENAI_API_KEY="your_valid_api_key"
```

2. Check your account's usage limits at OpenAI dashboard

3. Handle API errors in code:
```python
import openai

try:
    transcript = openai.Audio.transcribe("whisper-1", audio_file)
except openai.error.RateLimitError:
    print("Rate limit exceeded. Please try again later.")
except openai.error.AuthenticationError:
    print("Invalid API key. Please check your OPENAI_API_KEY.")
```

## Navigation Issues

### Costmap Problems

**Problem:** Robot doesn't navigate properly or gets stuck
**Solutions:**
1. Check sensor data:
```bash
# Verify laser scan data
ros2 topic echo /scan
```

2. Adjust costmap parameters:
```yaml
# In your costmap configuration
inflation_layer:
  plugin: "nav2_costmap_2d::InflationLayer"
  cost_scaling_factor: 3.0  # Increase for more cautious navigation
  inflation_radius: 0.55    # Increase for wider safety margin
```

### Localization Issues

**Problem:** Robot doesn't know where it is in the map
**Solutions:**
1. Ensure proper initial pose estimation:
```bash
# In RViz, use the "2D Pose Estimate" tool to set initial position
```

2. Check odometry quality:
```bash
# Verify odometry data
ros2 topic echo /odom
```

## Computer Vision Issues

### Perception Problems

**Problem:** Object detection doesn't work reliably
**Solutions:**
1. Ensure proper lighting conditions
2. Check camera calibration:
```bash
# Verify camera intrinsic parameters
ros2 param get /camera_node camera_matrix
```

3. Improve detection thresholds:
```python
# In your detection code
detection_threshold = 0.7  # Increase for fewer false positives
```

### Training Data Issues

**Problem:** Model trained on synthetic data doesn't work on real data
**Solutions:**
1. Implement domain randomization in simulation
2. Collect and label some real-world data to augment training
3. Use techniques like fine-tuning on real data

## Docker-Specific Issues

### Container Access Problems

**Problem:** Cannot enter running container
**Solution:**
```bash
# List running containers
docker ps

# Enter specific container
docker exec -it container_name bash
```

### Resource Limitations

**Problem:** Docker runs out of memory or CPU
**Solution:**
```bash
# Increase Docker resources in Docker Desktop settings
# Or limit resource usage in your applications
```

### Network Issues

**Problem:** ROS nodes in different containers cannot communicate
**Solutions:**
1. Use host networking:
```bash
docker run --network host your-image
```

2. Or ensure ROS_DOMAIN_ID is consistent:
```bash
export ROS_DOMAIN_ID=0
```

## Hardware-Specific Issues

### Jetson Orin Problems

**Problem:** Jetson overheats during intensive computation
**Solutions:**
1. Ensure proper cooling (fan/sink)
2. Monitor temperature:
```bash
sudo tegrastats  # On Jetson devices
```

3. Reduce computational load temporarily

### RealSense Camera Issues

**Problem:** RealSense camera not detected
**Solutions:**
1. Check USB connection (requires USB 3.0 for full features)
2. Install RealSense tools:
```bash
sudo apt install ros-humble-realsense2-camera
```

## Quiz

1. What command should you run if `colcon build` fails with missing dependencies?
2. How can you check if ROS topics are being published correctly?
3. What should you do if your robot gets stuck during navigation?

## Summary

This troubleshooting guide covers the most common issues you might encounter during the course. Remember to:

1. Check that all required services are running
2. Verify your environment variables are set correctly
3. Ensure proper hardware connections
4. Review error messages carefully for clues
5. Consult community resources when needed

Most issues in robotics have logical solutions based on understanding the system architecture. Take time to understand the error messages and trace the problem to its source.