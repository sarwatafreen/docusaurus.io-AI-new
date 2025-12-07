---
sidebar_position: 1
---

# Lesson 1: Your First Robot "Hello World"

In this lesson, we'll create our first robot in ROS 2. This is the "Hello World" of robotics, where we'll set up a minimal robot that can be launched and visualized in a simulation environment.

## Prerequisites
- ROS 2 Humble Hawksbill installed (or your preferred ROS 2 distribution)
- Basic understanding of Linux command line
- Text editor or IDE for development

## What You'll Build
By the end of this lesson, you'll have:
1. Created a minimal ROS 2 package for your robot
2. Defined a simple robot model in URDF (Unified Robot Description Format)
3. Launched a minimal robot simulation
4. Visualized your robot in RViz

## Setting Up Your Workspace

First, let's create a workspace for our robot project:

```bash
mkdir -p ~/physical_ai_ws/src
cd ~/physical_ai_ws
```

## Creating Your Robot Package

Now we'll create a package for our robot:

```bash
cd src
ros2 pkg create --build-type ament_python my_first_robot --dependencies rclpy std_msgs geometry_msgs
```

## Understanding the Package Structure

After creating the package, you'll see this structure:

```
my_first_robot/
├── CMakeLists.txt
├── package.xml
├── setup.cfg
├── setup.py
└── my_first_robot/
    └── __init__.py
```

## Creating the Robot Description (URDF)

Let's create a simple robot model. Create a new directory for robot descriptions:

```bash
mkdir -p my_first_robot/urdf
```

Then create `my_first_robot/urdf/simple_robot.urdf`:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <!-- Left wheel -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="-0.15 -0.2 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>

  <!-- Right wheel -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="-0.15 0.2 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>
</robot>
```

## Launching Your Robot

*Note: This section would include a GIF demonstrating the robot in RViz. The actual GIF would show the robot model loaded in RViz with its different links visible.*

Create a launch directory:

```bash
mkdir -p my_first_robot/launch
```

Create `my_first_robot/launch/simple_robot.launch.py`:

```python
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('my_first_robot')
    
    # Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command([
                'xacro ', os.path.join(pkg_share, 'urdf', 'simple_robot.urdf')
            ])
        }]
    )
    
    # Joint State Publisher node
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )
    
    # RViz node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'config.rviz')]
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rviz
    ])
```

## Creating the RViz Configuration

Create an rviz directory:

```bash
mkdir -p my_first_robot/rviz
```

Create `my_first_robot/rviz/config.rviz`:

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
  Global Options:
    Fixed Frame: base_link
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
```

## Building and Running

Build your workspace:

```bash
cd ~/physical_ai_ws
colcon build
source install/setup.bash
```

Run your first robot:

```bash
ros2 launch my_first_robot simple_robot.launch.py
```

## Quiz

1. What does URDF stand for?
2. What is the purpose of the robot_state_publisher node?
3. Why do we need both robot_state_publisher and joint_state_publisher?

## Runnable Code Example

Here's a simple ROS 2 publisher node you can run to understand the basics:

```python
# publisher_member_function.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

To run this code:
1. Create a new ROS 2 package: `ros2 pkg create --build-type ament_python my_publisher_pkg`
2. Save this code as `my_publisher_pkg/my_publisher_pkg/publisher_member_function.py`
3. Make it executable: `chmod +x my_publisher_pkg/my_publisher_pkg/publisher_member_function.py`
4. Build: `colcon build --packages-select my_publisher_pkg`
5. Run: `ros2 run my_publisher_pkg publisher_member_function`

## Summary

In this lesson, you created your first ROS 2 robot package with a simple URDF model and launched it in RViz. This forms the foundation for all future robot development in this course. In the next lesson, we'll learn to control this robot with keyboard commands!

## Quiz

import Quiz from '@site/src/components/Quiz';

<Quiz
  questions={[
    {
      question: "What is the purpose of the robot_state_publisher node?",
      options: ["To publish sensor data", "To publish the state of robot joints", "To control robot movement", "To manage ROS topics"],
      correctAnswer: 1
    },
    {
      question: "What does URDF stand for?",
      options: ["Universal Robot Description Format", "Unified Robot Development Framework", "User Robot Design Format", "Ultra Robot Definition File"],
      correctAnswer: 0
    },
    {
      question: "Which command launches a ROS 2 package?",
      options: ["ros2 run", "ros2 launch", "ros2 start", "ros2 execute"],
      correctAnswer: 1
    }
  ]}
/>