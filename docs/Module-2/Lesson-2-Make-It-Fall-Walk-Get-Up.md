---
sidebar_position: 2
---

# Lesson 2: Make It Fall, Walk, and Get Up

In this lesson, we'll implement dynamic behaviors for our humanoid robot. We'll start with basic stability, then teach our robot to walk, and finally implement recovery behaviors so it can get back up when it falls. This lesson combines physics simulation with robot control.

## Prerequisites
- Completed Module 1: ROS 2 basics
- Module 2, Lesson 1: Spawn your robot in Gazebo
- Understanding of joint control and physics simulation

## What You'll Build
By the end of this lesson, you'll have:
1. Implemented balance control for your humanoid
2. Created a walking gait for your robot
3. Developed a fall recovery behavior
4. Tested these behaviors in Gazebo simulation

## Understanding Humanoid Balance

Balancing a humanoid robot is a complex challenge that involves:
- Maintaining the Center of Mass (CoM) over the support polygon
- Using feedback from simulated sensors (IMU, joint encoders)
- Implementing control algorithms like PID controllers
- Adjusting joint positions to maintain stability

## Creating Balance Control

First, let's create a balance controller. Create `my_first_robot/balance_controller.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray
import numpy as np
from collections import deque
import math


class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')
        
        # Subscribers for IMU and joint states
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Publishers for joint commands
        self.left_hip_pub = self.create_publisher(
            Float64MultiArray, '/torso_to_left_hip_controller/position_commands', 10)
        self.right_hip_pub = self.create_publisher(
            Float64MultiArray, '/torso_to_right_hip_controller/position_commands', 10)
        self.left_knee_pub = self.create_publisher(
            Float64MultiArray, '/left_knee_controller/position_commands', 10)
        self.right_knee_pub = self.create_publisher(
            Float64MultiArray, '/right_knee_controller/position_commands', 10)
        
        # Internal state
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.joint_positions = {}
        self.target_joint_positions = {}
        
        # PID controller parameters for balance
        self.kp = 2.0  # Proportional gain
        self.ki = 0.1  # Integral gain  
        self.kd = 0.5  # Derivative gain
        
        # PID state variables
        self.prev_error = 0.0
        self.integral = 0.0
        
        # Timer for control loop
        self.timer = self.create_timer(0.01, self.balance_callback)  # 100Hz

    def imu_callback(self, msg):
        # Convert quaternion to Euler angles
        w, x, y, z = msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        self.roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            self.pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            self.pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def joint_state_callback(self, msg):
        # Store current joint positions
        for i, name in enumerate(msg.name):
            self.joint_positions[name] = msg.position[i]

    def balance_callback(self):
        # Calculate balance correction based on IMU data
        # For simplicity, we'll focus on correcting pitch to keep robot upright
        target_pitch = 0.0  # We want to stay upright
        
        # Calculate error
        error = target_pitch - self.pitch
        
        # PID control
        self.integral += error * 0.01  # dt = 0.01
        derivative = (error - self.prev_error) / 0.01
        
        # Calculate correction
        correction = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        # Apply correction to hip joints to maintain balance
        left_hip_correction = correction
        right_hip_correction = correction
        
        # Publish commands
        left_hip_msg = Float64MultiArray()
        left_hip_msg.data = [left_hip_correction]
        self.left_hip_pub.publish(left_hip_msg)
        
        right_hip_msg = Float64MultiArray()
        right_hip_msg.data = [right_hip_correction]
        self.right_hip_pub.publish(right_hip_msg)
        
        # Update previous error
        self.prev_error = error


def main():
    rclpy.init()
    balance_controller = BalanceController()
    
    try:
        rclpy.spin(balance_controller)
    except KeyboardInterrupt:
        pass
    finally:
        balance_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Creating a Walking Controller

Now let's create a simple walking controller. Create `my_first_robot/walking_controller.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time


class WalkingController(Node):
    def __init__(self):
        super().__init__('walking_controller')
        
        # Publishers for joint commands
        self.left_hip_pub = self.create_publisher(
            Float64MultiArray, '/torso_to_left_hip_controller/position_commands', 10)
        self.right_hip_pub = self.create_publisher(
            Float64MultiArray, '/torso_to_right_hip_controller/position_commands', 10)
        self.left_knee_pub = self.create_publisher(
            Float64MultiArray, '/left_knee_controller/position_commands', 10)
        self.right_knee_pub = self.create_publisher(
            Float64MultiArray, '/right_knee_controller/position_commands', 10)
        self.left_ankle_pub = self.create_publisher(
            Float64MultiArray, '/left_ankle_controller/position_commands', 10)
        self.right_ankle_pub = self.create_publisher(
            Float64MultiArray, '/right_ankle_controller/position_commands', 10)
        
        # Walking state
        self.phase = 0.0
        self.walking = False
        self.step_frequency = 1.0  # steps per second
        
        # Timer for walking control
        self.timer = self.create_timer(0.01, self.walk_callback)  # 100Hz

    def start_walking(self):
        self.walking = True

    def stop_walking(self):
        self.walking = False
        # Return to neutral position
        self.publish_neutral_position()

    def publish_neutral_position(self):
        # Publish neutral position for all joints
        neutral_msg = Float64MultiArray()
        neutral_msg.data = [0.0]  # Neutral hip position
        
        self.left_hip_pub.publish(neutral_msg)
        self.right_hip_pub.publish(neutral_msg)
        self.left_knee_pub.publish(neutral_msg)
        self.right_knee_pub.publish(neutral_msg)
        self.left_ankle_pub.publish(neutral_msg)
        self.right_ankle_pub.publish(neutral_msg)

    def walk_callback(self):
        if not self.walking:
            return
            
        # Update phase based on time
        self.phase += 2 * math.pi * self.step_frequency * 0.01  # dt = 0.01
        
        # Calculate joint angles for walking gait
        # This is a simplified walking pattern
        left_hip_angle = 0.2 * math.sin(self.phase)
        right_hip_angle = 0.2 * math.sin(self.phase + math.pi)  # Opposite phase
        
        left_knee_angle = 0.3 * math.sin(self.phase * 2)  # Double frequency
        right_knee_angle = 0.3 * math.sin(self.phase * 2 + math.pi)  # Opposite phase
        
        # Ankle adjustments for balance
        left_ankle_angle = 0.1 * math.sin(self.phase)
        right_ankle_angle = 0.1 * math.sin(self.phase + math.pi)
        
        # Publish joint commands
        left_hip_msg = Float64MultiArray()
        left_hip_msg.data = [left_hip_angle]
        self.left_hip_pub.publish(left_hip_msg)
        
        right_hip_msg = Float64MultiArray()
        right_hip_msg.data = [right_hip_angle]
        self.right_hip_pub.publish(right_hip_msg)
        
        left_knee_msg = Float64MultiArray()
        left_knee_msg.data = [left_knee_angle]
        self.left_knee_pub.publish(left_knee_msg)
        
        right_knee_msg = Float64MultiArray()
        right_knee_msg.data = [right_knee_angle]
        self.right_knee_pub.publish(right_knee_msg)
        
        left_ankle_msg = Float64MultiArray()
        left_ankle_msg.data = [left_ankle_angle]
        self.left_ankle_pub.publish(left_ankle_msg)
        
        right_ankle_msg = Float64MultiArray()
        right_ankle_msg.data = [right_ankle_angle]
        self.right_ankle_pub.publish(right_ankle_msg)
        
        # Reset phase to avoid overflow
        if self.phase > 2 * math.pi:
            self.phase -= 2 * math.pi


def main():
    rclpy.init()
    walking_controller = WalkingController()
    
    # Start walking after a short delay
    def start_walking():
        walking_controller.start_walking()
        walking_controller.get_logger().info("Starting to walk...")
    
    # Use a timer to start walking after initialization
    timer = walking_controller.create_timer(2.0, start_walking)
    
    try:
        rclpy.spin(walking_controller)
    except KeyboardInterrupt:
        walking_controller.stop_walking()
        pass
    finally:
        walking_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Creating a Fall Recovery Controller

For the fall recovery behavior, create `my_first_robot/recovery_controller.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
import math
import time


class RecoveryController(Node):
    def __init__(self):
        super().__init__('recovery_controller')
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)
        
        # Publishers for joint commands
        self.left_hip_pub = self.create_publisher(
            Float64MultiArray, '/torso_to_left_hip_controller/position_commands', 10)
        self.right_hip_pub = self.create_publisher(
            Float64MultiArray, '/torso_to_right_hip_controller/position_commands', 10)
        self.left_knee_pub = self.create_publisher(
            Float64MultiArray, '/left_knee_controller/position_commands', 10)
        self.right_knee_pub = self.create_publisher(
            Float64MultiArray, '/right_knee_controller/position_commands', 10)
        self.left_ankle_pub = self.create_publisher(
            Float64MultiArray, '/left_ankle_controller/position_commands', 10)
        self.right_ankle_pub = self.create_publisher(
            Float64MultiArray, '/right_ankle_controller/position_commands', 10)
        self.left_shoulder_pub = self.create_publisher(
            Float64MultiArray, '/torso_to_left_shoulder_controller/position_commands', 10)
        self.right_shoulder_pub = self.create_publisher(
            Float64MultiArray, '/torso_to_right_shoulder_controller/position_commands', 10)
        
        # Internal state
        self.roll = 0.0
        self.pitch = 0.0
        self.fallen = False
        self.recovery_phase = 0  # 0: not recovering, 1: getting to prone, 2: pushing up, 3: standing
        
        # Timer for checking fall status
        self.fall_check_timer = self.create_timer(0.1, self.check_fall_status)
        
        # Timer for recovery sequence
        self.recovery_timer = None
        self.recovery_start_time = 0

    def imu_callback(self, msg):
        # Convert quaternion to Euler angles
        w, x, y, z = msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        self.roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            self.pitch = math.copysign(math.pi / 2, sinp)
        else:
            self.pitch = math.asin(sinp)

    def check_fall_status(self):
        # Determine if the robot has fallen based on orientation
        # If pitch is too far from upright (0), consider it fallen
        if abs(self.pitch) > 1.0:  # 1.0 radians is about 57 degrees
            if not self.fallen:
                self.fallen = True
                self.get_logger().info("Robot has fallen! Initiating recovery sequence.")
                self.start_recovery()
        else:
            self.fallen = False

    def start_recovery(self):
        self.recovery_phase = 1  # Start getting to prone position
        self.recovery_start_time = time.time()
        
        # Start recovery timer
        self.recovery_timer = self.create_timer(0.01, self.execute_recovery)

    def execute_recovery(self):
        if self.recovery_phase == 1:
            # Phase 1: Get to prone position
            self.move_to_prone()
            
            # After a delay, move to next phase
            if time.time() - self.recovery_start_time > 2.0:
                self.recovery_phase = 2
                self.recovery_start_time = time.time()
                
        elif self.recovery_phase == 2:
            # Phase 2: Push up using arms
            self.push_up_with_arms()
            
            if time.time() - self.recovery_start_time > 2.0:
                self.recovery_phase = 3
                self.recovery_start_time = time.time()
                
        elif self.recovery_phase == 3:
            # Phase 3: Stand up
            self.stand_up()
            
            if time.time() - self.recovery_start_time > 3.0:
                self.recovery_phase = 0  # Recovery complete
                self.get_logger().info("Recovery complete! Robot is standing.")
                
                # Stop the recovery timer
                self.recovery_timer.destroy()

    def move_to_prone(self):
        # Move arms out to help stabilize and get to prone
        left_shoulder_msg = Float64MultiArray()
        left_shoulder_msg.data = [0.5]  # Move arm out
        self.left_shoulder_pub.publish(left_shoulder_msg)
        
        right_shoulder_msg = Float64MultiArray()
        right_shoulder_msg.data = [-0.5]  # Move arm out
        self.right_shoulder_pub.publish(right_shoulder_msg)
        
        # Keep legs neutral
        neutral_msg = Float64MultiArray()
        neutral_msg.data = [0.0]
        self.left_hip_pub.publish(neutral_msg)
        self.right_hip_pub.publish(neutral_msg)
        self.left_knee_pub.publish(neutral_msg)
        self.right_knee_pub.publish(neutral_msg)
        self.left_ankle_pub.publish(neutral_msg)
        self.right_ankle_pub.publish(neutral_msg)

    def push_up_with_arms(self):
        # Use arms to push body up
        left_shoulder_msg = Float64MultiArray()
        left_shoulder_msg.data = [1.0]  # Push with arm
        self.left_shoulder_pub.publish(left_shoulder_msg)
        
        right_shoulder_msg = Float64MultiArray()
        right_shoulder_msg.data = [-1.0]  # Push with arm
        self.right_shoulder_pub.publish(right_shoulder_msg)

    def stand_up(self):
        # Move to standing position
        left_hip_msg = Float64MultiArray()
        left_hip_msg.data = [0.0]  # Neutral position
        self.left_hip_pub.publish(left_hip_msg)
        
        right_hip_msg = Float64MultiArray()
        right_hip_msg.data = [0.0]  # Neutral position
        self.right_hip_pub.publish(right_hip_msg)
        
        left_knee_msg = Float64MultiArray()
        left_knee_msg.data = [-0.3]  # Slightly bent
        self.left_knee_pub.publish(left_knee_msg)
        
        right_knee_msg = Float64MultiArray()
        right_knee_msg.data = [-0.3]  # Slightly bent
        self.right_knee_pub.publish(right_knee_msg)
        
        left_ankle_msg = Float64MultiArray()
        left_ankle_msg.data = [0.0]  # Neutral
        self.left_ankle_pub.publish(left_ankle_msg)
        
        right_ankle_msg = Float64MultiArray()
        right_ankle_msg.data = [0.0]  # Neutral
        self.right_ankle_pub.publish(right_ankle_msg)
        
        # Move arms to neutral
        neutral_arm_msg = Float64MultiArray()
        neutral_arm_msg.data = [0.0]
        self.left_shoulder_pub.publish(neutral_arm_msg)
        self.right_shoulder_pub.publish(neutral_arm_msg)


def main():
    rclpy.init()
    recovery_controller = RecoveryController()
    
    try:
        rclpy.spin(recovery_controller)
    except KeyboardInterrupt:
        pass
    finally:
        recovery_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Creating a Combined Controller Launch File

Create `my_first_robot/launch/behavior_controllers.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        # Balance controller
        Node(
            package='my_first_robot',
            executable='balance_controller',
            name='balance_controller',
            output='screen',
        ),
        
        # Walking controller
        Node(
            package='my_first_robot',
            executable='walking_controller',
            name='walking_controller',
            output='screen',
        ),
        
        # Recovery controller
        Node(
            package='my_first_robot',
            executable='recovery_controller',
            name='recovery_controller',
            output='screen',
        ),
    ])
```

## Adding an IMU to the Robot Model

We need to add an IMU sensor to our robot model. Update `my_first_robot/urdf/humanoid.gazebo.xacro`:

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

  <!-- Gazebo IMU Sensor Plugin -->
  <gazebo reference="head">
    <sensor name="imu" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </z>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>
      <ros>
        <namespace>/</namespace>
        <remapping>~/out:=imu</remapping>
      </ros>
    </sensor>
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

## Testing the Behaviors

Build your workspace:

```bash
cd ~/physical_ai_ws
colcon build --packages-select my_first_robot
source install/setup.bash
```

Launch Gazebo with the robot and controllers:

```bash
# Terminal 1: Launch Gazebo with the robot
ros2 launch my_first_robot humanoid_gazebo.launch.py

# Terminal 2: Launch the behavior controllers
ros2 launch my_first_robot behavior_controllers.launch.py
```

## Quiz

1. What are the three phases of the fall recovery sequence?
2. How does the IMU sensor help with balance control?
3. What is the purpose of PID control in robot balance?

## Summary

In this lesson, you've implemented complex dynamic behaviors for your humanoid robot. You created controllers for balance, walking, and fall recovery. These behaviors are essential for creating a functional humanoid robot that can operate in the real world.

In the next lesson, we'll add realistic sensors like cameras, LiDAR, and IMU to our robot model!