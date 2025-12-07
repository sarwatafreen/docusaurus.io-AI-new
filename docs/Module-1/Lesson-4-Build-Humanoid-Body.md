---
sidebar_position: 4
---

# Lesson 4: Build Your Own Humanoid Body (URDF)

In this lesson, we'll create a detailed humanoid robot model using URDF (Unified Robot Description Format). This will be the physical representation of your robot that can be used in simulation and later with real hardware.

## Prerequisites
- Completed Lessons 1-3 of Module 1
- Understanding of ROS 2 topics, services, and basic robot modeling

## What You'll Build
By the end of this lesson, you'll have:
1. Created a complete humanoid robot model in URDF
2. Added physical properties for simulation
3. Included joint limits and safety constraints
4. Verified your model with RViz and Gazebo simulation

## Understanding URDF Structure

URDF is an XML-based format for representing robot models. A URDF file contains:

- **Links**: Rigid parts of the robot (e.g., body, arms, legs)
- **Joints**: Connections between links (e.g., hinges, prismatic joints)
- **Visual**: How the robot looks (geometry, materials, colors)
- **Collision**: How the robot interacts physically (collision shapes)
- **Inertial**: Physical properties for simulation (mass, center of mass, inertia)

## Creating a Humanoid Robot Model

Create `my_first_robot/urdf/humanoid.urdf`:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Base link - represents the torso -->
  <link name="torso">
    <visual>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <capsule length="0.8" radius="0.15"/>
      </geometry>
      <material name="light_grey">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <capsule length="0.8" radius="0.15"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <inertia ixx="0.6" ixy="0.0" ixz="0.0" iyy="0.6" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="torso_to_head" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 1.0" rpy="0 0 0"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    
    <collision>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.015" ixy="0.0" ixz="0.0" iyy="0.015" iyz="0.0" izz="0.015"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="torso_to_left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.7" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <origin xyz="0 -0.15 0" rpy="1.57 0 0"/>
      <geometry>
        <capsule length="0.3" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 -0.15 0" rpy="1.57 0 0"/>
      <geometry>
        <capsule length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 -0.15 0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.0025"/>
    </inertial>
  </link>

  <joint name="left_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 -0.3 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <origin xyz="0 -0.1 0" rpy="1.57 0 0"/>
      <geometry>
        <capsule length="0.2" radius="0.04"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 -0.1 0" rpy="1.57 0 0"/>
      <geometry>
        <capsule length="0.2" radius="0.04"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 -0.1 0"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right Arm (mirror left arm) -->
  <joint name="torso_to_right_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.2 0 0.7" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

  <link name="right_upper_arm">
    <visual>
      <origin xyz="0 0.15 0" rpy="1.57 0 0"/>
      <geometry>
        <capsule length="0.3" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0.15 0" rpy="1.57 0 0"/>
      <geometry>
        <capsule length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0.15 0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.0025"/>
    </inertial>
  </link>

  <joint name="right_elbow" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0.3 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

  <link name="right_lower_arm">
    <visual>
      <origin xyz="0 0.1 0" rpy="1.57 0 0"/>
      <geometry>
        <capsule length="0.2" radius="0.04"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0.1 0" rpy="1.57 0 0"/>
      <geometry>
        <capsule length="0.2" radius="0.04"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0.1 0"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Left Leg -->
  <joint name="torso_to_left_hip" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_leg"/>
    <origin xyz="0.1 -0.1 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="200" velocity="1.0"/>
  </joint>

  <link name="left_upper_leg">
    <visual>
      <origin xyz="0 -0.2 0" rpy="1.57 0 0"/>
      <geometry>
        <capsule length="0.4" radius="0.06"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 -0.2 0" rpy="1.57 0 0"/>
      <geometry>
        <capsule length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 -0.2 0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_knee" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0 -0.4 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="0.5" effort="200" velocity="1.0"/>
  </joint>

  <link name="left_lower_leg">
    <visual>
      <origin xyz="0 -0.15 0" rpy="1.57 0 0"/>
      <geometry>
        <capsule length="0.3" radius="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 -0.15 0" rpy="1.57 0 0"/>
      <geometry>
        <capsule length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 -0.15 0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.0075"/>
    </inertial>
  </link>

  <joint name="left_ankle" type="revolute">
    <parent link="left_lower_leg"/>
    <child link="left_foot"/>
    <origin xyz="0 -0.3 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1.0"/>
  </joint>

  <link name="left_foot">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    
    <collision>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.0005" ixy="0.0" ixz="0.0" iyy="0.0015" iyz="0.0" izz="0.0015"/>
    </inertial>
  </link>

  <!-- Right Leg (mirror left leg) -->
  <joint name="torso_to_right_hip" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_leg"/>
    <origin xyz="0.1 0.1 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="200" velocity="1.0"/>
  </joint>

  <link name="right_upper_leg">
    <visual>
      <origin xyz="0 0.2 0" rpy="1.57 0 0"/>
      <geometry>
        <capsule length="0.4" radius="0.06"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0.2 0" rpy="1.57 0 0"/>
      <geometry>
        <capsule length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0.2 0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="right_knee" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0 0.4 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="0.5" effort="200" velocity="1.0"/>
  </joint>

  <link name="right_lower_leg">
    <visual>
      <origin xyz="0 0.15 0" rpy="1.57 0 0"/>
      <geometry>
        <capsule length="0.3" radius="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0.15 0" rpy="1.57 0 0"/>
      <geometry>
        <capsule length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0.15 0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.0075"/>
    </inertial>
  </link>

  <joint name="right_ankle" type="revolute">
    <parent link="right_lower_leg"/>
    <child link="right_foot"/>
    <origin xyz="0 0.3 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1.0"/>
  </joint>

  <link name="right_foot">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    
    <collision>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.0005" ixy="0.0" ixz="0.0" iyy="0.0015" iyz="0.0" izz="0.0015"/>
    </inertial>
  </link>
</robot>
```

## Adding Controllers for the Humanoid

Create a controllers configuration file: `my_first_robot/config/humanoid_controllers.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    torso_to_left_shoulder_controller:
      type: position_controllers/JointPositionController

    left_elbow_controller:
      type: position_controllers/JointPositionController

    torso_to_right_shoulder_controller:
      type: position_controllers/JointPositionController

    right_elbow_controller:
      type: position_controllers/JointPositionController

    torso_to_left_hip_controller:
      type: position_controllers/JointPositionController

    left_knee_controller:
      type: position_controllers/JointPositionController

    left_ankle_controller:
      type: position_controllers/JointPositionController

    torso_to_right_hip_controller:
      type: position_controllers/JointPositionController

    right_knee_controller:
      type: position_controllers/JointPositionController

    right_ankle_controller:
      type: position_controllers/JointPositionController
```

## Creating a Launch File for the Humanoid

Create `my_first_robot/launch/humanoid.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Get paths to URDF and controller config
    robot_description_path = PathJoinSubstitution([
        FindPackageShare('my_first_robot'),
        'urdf',
        'humanoid.urdf'
    ])
    
    controllers_config_path = PathJoinSubstitution([
        FindPackageShare('my_first_robot'),
        'config',
        'humanoid_controllers.yaml'
    ])

    # Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': PathJoinSubstitution([
                FindPackageShare('my_first_robot'),
                'urdf',
                'humanoid.urdf'
            ])
        }],
        remappings=[
            ('/joint_states', 'demo_joint_states')
        ]
    )

    # Joint State Publisher node (for demonstration)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'source_list': ['demo_joint_states']
        }]
    )

    # Joint State Publisher GUI (for manual control)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )

    # RViz node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('my_first_robot'),
            'rviz',
            'humanoid.rviz'
        ])],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # Controller Manager (for simulation)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_config_path],
        remappings=[
            ('/diff_drive_controller/cmd_vel_unstamped', '/cmd_vel'),
        ]
    )

    # Spawn controllers
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Define spawn controller events
    spawn_controllers = []
    controller_names = [
        'torso_to_left_shoulder_controller',
        'left_elbow_controller',
        'torso_to_right_shoulder_controller',
        'right_elbow_controller',
        'torso_to_left_hip_controller',
        'left_knee_controller',
        'left_ankle_controller',
        'torso_to_right_hip_controller',
        'right_knee_controller',
        'right_ankle_controller'
    ]

    for controller in controller_names:
        spawn_controllers.append(
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[controller],
                parameters=[{'use_sim_time': use_sim_time}],
            )
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Use joint_state_publisher_gui for manual joint control'
        ),
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,
        rviz,
        controller_manager,
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager,
                on_start=[spawn_joint_state_broadcaster],
            )
        ),
        *spawn_controllers
    ])
```

## Creating an RViz Configuration for the Humanoid

Create `my_first_robot/rviz/humanoid.rviz`:

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Topic: /robot_description
      Alpha: 1.0
      Show Axes: false
      Show Trail: false
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: false
  Global Options:
    Fixed Frame: torso
    Frame Rate: 30
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Name: Orbit
      Target Frame: torso
      Value: Orbit (rviz)
      Distance: 2.5
      Pitch: 0.3
      Yaw: 0.5
```

## Testing Your Humanoid Model

Build and run your humanoid model:

```bash
cd ~/physical_ai_ws
colcon build --packages-select my_first_robot
source install/setup.bash

# Launch the humanoid
ros2 launch my_first_robot humanoid.launch.py
```

## Quiz

1. What are the three main components of a URDF model?
2. What is the difference between visual and collision properties in URDF?
3. What does the `revolute` joint type represent in URDF?

## Summary

In this lesson, you've built a complete humanoid robot model using URDF. This model includes the physical structure, visual representation, and physical properties needed for simulation. You also created controller configurations to manage the robot's joints.

This completes Module 1 of our Physical AI & Humanoid Robotics course. You now have a solid foundation in ROS 2 concepts and robot modeling. In Module 2, we'll simulate this robot in Gazebo and Unity environments!