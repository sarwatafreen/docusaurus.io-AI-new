---
sidebar_position: 1
---

# Lesson 1: Spawn Your Robot in Gazebo in 30 Seconds

Welcome to the world of robot simulation! In this lesson, we'll take the humanoid robot we created in Module 1 and bring it to life in Gazebo, a powerful physics-based simulation environment. By the end of this lesson, you'll have your robot standing in a realistic 3D world.

## Prerequisites
- Completed Module 1: ROS 2 basics
- Your humanoid robot model from Module 1
- Gazebo installed (Gazebo Garden or Fortress)

## What You'll Achieve
By the end of this lesson, you'll have:
1. Installed and configured Gazebo for robot simulation
2. Integrated your humanoid model with Gazebo physics
3. Spawned your robot in a Gazebo environment
4. Visualized and interacted with your robot in simulation

## Understanding Gazebo

Gazebo is a 3D simulation environment that provides:
- High-fidelity physics simulation
- Realistic rendering and lighting
- Sensor simulation (cameras, LiDAR, IMU, etc.)
- Multiple physics engines (ODE, Bullet, Simbody, DART)

## Installing Gazebo

For ROS 2 Humble on Ubuntu:
```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
```

## Creating a Gazebo-Compatible Robot Model

We need to enhance our URDF model with Gazebo-specific tags. Create `my_first_robot/urdf/humanoid.gazebo.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Gazebo-specific plugins and materials -->
  <gazebo reference="torso">
    <material>Gazebo/Gray</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <self_collide>0</self_collide>
  </gazebo>

  <gazebo reference="head">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="left_upper_arm">
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="left_lower_arm">
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="right_upper_arm">
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="right_lower_arm">
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="left_upper_leg">
    <material>Gazebo/Green</material>
  </gazebo>

  <gazebo reference="left_lower_leg">
    <material>Gazebo/Green</material>
  </gazebo>

  <gazebo reference="left_foot">
    <material>Gazebo/Black</material>
  </gazebo>

  <gazebo reference="right_upper_leg">
    <material>Gazebo/Green</material>
  </gazebo>

  <gazebo reference="right_lower_leg">
    <material>Gazebo/Green</material>
  </gazebo>

  <gazebo reference="right_foot">
    <material>Gazebo/Black</material>
  </gazebo>

  <!-- Gazebo ROS2 Control Plugin -->
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(find my_first_robot)/config/humanoid_controllers.yaml</parameters>
    </plugin>
  </gazebo>

  <!-- Gazebo world plugin for time control -->
  <gazebo>
    <plugin name="gazebo_ros_init" filename="libgazebo_ros_init.so">
      <ros>
        <namespace>/gazebo</namespace>
      </ros>
      <update_rate>100</update_rate>
    </plugin>
  </gazebo>
</robot>
```

## Creating a Complete Gazebo Robot Model

Now let's create a combined URDF that includes both the original model and Gazebo-specific elements. Create `my_first_robot/urdf/humanoid_gazebo.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot">
  <!-- Include the base humanoid model -->
  <xacro:include filename="humanoid.urdf"/>

  <!-- Include Gazebo-specific elements -->
  <xacro:include filename="humanoid.gazebo.xacro"/>

  <!-- Gazebo world plugin for physics -->
  <gazebo>
    <plugin filename="libgazebo_ros_init.so" name="gazebo_ros_init">
      <ros>
        <namespace>/gazebo</namespace>
      </ros>
    </plugin>
  </gazebo>

  <!-- Add ground plane for robot to stand on -->
  <link name="world"/>
  <joint name="world_to_base" type="fixed">
    <parent link="world"/>
    <child link="torso"/>
    <origin xyz="0 0 0.8" rpy="0 0 0"/>
  </joint>
</robot>
```

## Creating a Gazebo Launch File

Create `my_first_robot/launch/humanoid_gazebo.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    headless = LaunchConfiguration('headless', default='false')

    # Paths
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_share = get_package_share_directory('my_first_robot')
    
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'verbose': 'false',
            'headless': headless,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': PathJoinSubstitution([
                FindPackageShare('my_first_robot'),
                'urdf',
                'humanoid_gazebo.urdf.xacro'
            ])
        }]
    )

    # Spawn entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'humanoid_robot',
            '-x', '0', '-y', '0', '-z', '1.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Use headless mode for Gazebo if true'
        ),
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
```

## Installing Required Dependencies

For Gazebo ROS2 control, you'll need additional packages:

```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
```

## Running Your Robot in Gazebo

Build your workspace:

```bash
cd ~/physical_ai_ws
colcon build --packages-select my_first_robot
source install/setup.bash
```

Launch Gazebo with your robot:

```bash
ros2 launch my_first_robot humanoid_gazebo.launch.py
```

## Controlling Your Robot in Simulation

To move the robot joints in simulation, you can publish commands to the joint controllers:

```bash
# Set the left shoulder joint to 0.5 radians
ros2 topic pub /torso_to_left_shoulder_controller/position_commands std_msgs/msg/Float64MultiArray "data: [0.5]"

# Set multiple joints
ros2 topic pub /torso_to_left_shoulder_controller/position_commands std_msgs/msg/Float64MultiArray "data: [0.2]"
ros2 topic pub /torso_to_right_shoulder_controller/position_commands std_msgs/msg/Float64MultiArray "data: [-0.2]"
```

## Quiz

1. What is the purpose of the gazebo_ros2_control plugin?
2. Why do we use xacro instead of plain URDF for simulation?
3. What ROS 2 package provides the spawn_entity node?

## Summary

In this lesson, you've successfully integrated your humanoid robot model with Gazebo simulation. You learned how to enhance your URDF with Gazebo-specific elements and launch your robot in a 3D physics environment. This is a crucial step toward testing and developing robot behaviors before deploying on real hardware.

In the next lesson, we'll make your robot walk, fall, and get back up in simulation!