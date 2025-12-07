---
sidebar_position: 3
---

# Lesson 3: Robot Maps Room by Itself (VSLAM)

In this lesson, we'll implement Visual Simultaneous Localization and Mapping (VSLAM) to enable our robot to perceive and navigate unknown environments. VSLAM allows robots to build a map of their surroundings while simultaneously determining their position within that map.

## Prerequisites
- Understanding of ROS 2 concepts
- Completed Module 3, Lessons 1-2
- Basic knowledge of computer vision
- Access to Isaac Sim or a real robot with cameras

## What You'll Build
By the end of this lesson, you'll have:
1. Integrated a VSLAM algorithm (ORB-SLAM3 or similar) with ROS 2
2. Created a 3D map of a simulated room
3. Implemented real-time localization using visual features
4. Visualized the map in RViz

## Understanding VSLAM

Visual SLAM combines computer vision and robotics to solve two fundamental problems:
- Localizing the robot in an unknown environment
- Simultaneously mapping the environment

Key components of VSLAM systems:
- Feature detection and matching
- Camera pose estimation
- Map building and optimization
- Loop closure detection

## Setting Up the VSLAM Pipeline

First, let's install the required packages:

```bash
# Install ORB-SLAM3 ROS 2 wrapper
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3
chmod +x build.sh
./build.sh

# Install ROS 2 wrapper
git clone https://github.com/Allopart/orb_slam_3_ros2.git
cd orb_slam_3_ros2
colcon build
```

## Running VSLAM in Isaac Sim

Create a launch file to run VSLAM with Isaac Sim:

```xml
<!-- vslam_pipeline.launch.py -->
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    config_file = LaunchConfiguration('config_file')
    vocabulary_file = LaunchConfiguration('vocabulary_file')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value='/path/to/orb_slam3/config.yaml'
        ),
        DeclareLaunchArgument(
            'vocabulary_file',
            default_value='/path/to/Vocabulary/ORBvoc.txt'
        ),
        
        # ORB-SLAM3 node
        Node(
            package='orb_slam3_ros2',
            executable='orb_slam3_ros2',
            name='orb_slam3',
            parameters=[
                config_file,
                vocabulary_file
            ],
            remappings=[
                ('/camera/image_raw', '/your_robot/camera/image'),
                ('/camera/camera_info', '/your_robot/camera/camera_info')
            ]
        ),
        
        # TF broadcasters if needed
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
        )
    ])
```

## Configuring the VSLAM System

Create a configuration file for ORB-SLAM3:

```yaml
# orb_slam3_config.yaml
%YAML:1.0
---
# Camera Parameters
Camera.fx: 517.306408
Camera.fy: 516.469215
Camera.cx: 318.643040
Camera.cy: 255.313989

Camera.k1: 0.262383
Camera.k2: -0.953104
Camera.p1: 0.001185
Camera.p2: 0.001999
Camera.k3: 1.163314

Camera.width: 640
Camera.height: 480

# Camera frames per second
Camera.fps: 30.0

# IR projector baseline times fx (aprox.)
Camera.bf: 40.0

# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
Camera.RGB: 1

# Tracking parameters
# Minimum orientation change between keyframes
ThRelocDistance: 0.1
# Minimum translation change between keyframes
ThRelocAngle: 10