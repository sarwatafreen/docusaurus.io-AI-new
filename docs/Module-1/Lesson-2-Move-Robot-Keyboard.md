---
sidebar_position: 2
---

# Lesson 2: Move the Robot with Keyboard

In this lesson, we'll learn how to control our robot using keyboard commands. This introduces us to ROS 2 topics and the publish-subscribe communication pattern that is fundamental to all ROS systems.

## Prerequisites
- Completed Lesson 1: Your First Robot "Hello World"
- Your robot should be visualized in RViz
- Basic understanding of ROS 2 concepts

## What You'll Build
By the end of this lesson, you'll have:
1. Created a teleop (teleoperation) node that publishes velocity commands
2. Controlled your robot's movement using keyboard inputs
3. Understood how ROS 2 topics work in practice

## Understanding Twist Messages

In ROS, robot movement is controlled using `Twist` messages. A `Twist` message contains linear and angular velocity components:

- `linear.x`: Forward/backward velocity
- `linear.y`: Left/right velocity (for holonomic robots)
- `linear.z`: Up/down velocity
- `angular.x`: Roll angular velocity
- `angular.y`: Pitch angular velocity
- `angular.z`: Yaw angular velocity (turning left/right)

## Creating the Teleop Node

Create a new file `my_first_robot/teleop_keyboard.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

msg = """
Control Your Robot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.speed = 0.5
        self.turn = 1.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.status = 0

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self, speed, turn):
        return f"currently:\tspeed {speed:.2f}\tturn {turn:.2f}"

    def run(self):
        self.settings = termios.tcgetattr(sys.stdin)
        
        print(msg)
        print(self.vels(self.speed, self.turn))

        while True:
            key = self.getKey()
            if key in moveBindings.keys():
                self.x = moveBindings[key][0]
                self.y = moveBindings[key][1]
                self.z = moveBindings[key][2]
                self.th = moveBindings[key][3]
            elif key in speedBindings.keys():
                self.speed = self.speed * speedBindings[key][0]
                self.turn = self.turn * speedBindings[key][1]

                print(self.vels(self.speed, self.turn))
                if (self.status == 14):
                    print(msg)
                self.status = (self.status + 1) % 15
            else:
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0
                self.th = 0.0
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = self.th * self.turn
            self.publisher.publish(twist)

        # Stop the robot when exiting
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main():
    rclpy.init()
    
    node = TeleopKeyboard()
    
    try:
        node.run()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Making the Script Executable

Make the script executable:

```bash
chmod +x my_first_robot/teleop_keyboard.py
```

## Adding Dependencies

Add the necessary dependencies to your `package.xml`:

```xml
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
<depend>sensor_msgs</depend>
```

## Creating a Launch File for Teleoperation

Create `my_first_robot/launch/teleop.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_first_robot',
            executable='teleop_keyboard',
            name='teleop_keyboard',
            output='screen',
        )
    ])
```

## Building and Running

Build your workspace again:

```bash
cd ~/physical_ai_ws
colcon build --packages-select my_first_robot
source install/setup.bash
```

Run your robot with keyboard control:

```bash
ros2 run my_first_robot teleop_keyboard
```

## Understanding the Code

The teleop node demonstrates key ROS 2 concepts:

1. **Publisher**: The node creates a publisher to send velocity commands to `/cmd_vel`
2. **Topics**: The `/cmd_vel` topic follows the ROS convention for velocity commands
3. **Message Types**: Uses the `Twist` message type from `geometry_msgs`
4. **Node Lifecycle**: Properly initializes and shuts down the ROS node

## Quiz

1. What is the purpose of a publisher in ROS 2?
2. Which ROS message type is used for velocity commands?
3. Why do we need to call `rclpy.init()` before creating a node?

## Summary

In this lesson, you've learned how to control your robot using keyboard commands and how to publish messages to ROS 2 topics. This fundamental pattern of publishing/subscribing to topics is used throughout robotics. In the next lesson, we'll explore ROS 2 services and more advanced communication patterns!