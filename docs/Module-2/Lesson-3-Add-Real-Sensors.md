---
sidebar_position: 3
---

# Lesson 3: Add Real Sensors (Camera, LiDAR, IMU)

In this lesson, we'll enhance our humanoid robot with realistic sensors that will allow it to perceive its environment. We'll add a camera for vision, a LiDAR for 3D mapping, and an IMU for orientation and acceleration. These sensors are essential for any real-world robot application.

## Prerequisites
- Completed Module 1: ROS 2 basics
- Module 2, Lessons 1-2: Gazebo simulation and robot behaviors
- Understanding of ROS 2 message types and topics

## What You'll Build
By the end of this lesson, you'll have:
1. Added a camera sensor to your humanoid robot
2. Integrated a LiDAR sensor for 3D perception
3. Enhanced the IMU for better orientation sensing
4. Visualized sensor data in RViz
5. Tested sensor functionality in simulation

## Understanding Robot Sensors

Robots perceive their environment through sensors. The main types of sensors for humanoid robots are:

- **Camera**: Visual perception for object recognition, navigation, and interaction
- **LiDAR**: 3D perception for mapping and obstacle detection
- **IMU**: Inertial measurement for orientation, acceleration, and angular velocity
- **Other sensors**: Force/torque sensors, tactile sensors, GPS, etc.

## Adding a Camera Sensor

First, let's add a camera to our robot. We'll mount it on the head of our humanoid. Update the camera section in `my_first_robot/urdf/humanoid.gazebo.xacro`:

```xml
  <!-- Gazebo Camera Sensor Plugin -->
  <gazebo reference="head">
    <sensor name="camera" type="camera">
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <camera name="head_camera">
        <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>30</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>/</namespace>
          <remapping>~/image_raw:=/camera/image_raw</remapping>
          <remapping>~/camera_info:=/camera/camera_info</remapping>
        </ros>
        <camera_name>head_camera</camera_name>
        <frame_name>camera_frame</frame_name>
        <hack_baseline>0.07</hack_baseline>
        <distortion_k1>0.0</distortion_k1>
        <distortion_k2>0.0</distortion_k2>
        <distortion_k3>0.0</distortion_k3>
        <distortion_t1>0.0</distortion_t1>
        <distortion_t2>0.0</distortion_t2>
      </plugin>
    </sensor>
  </gazebo>
```

We also need to add a camera link to the URDF. Update `my_first_robot/urdf/humanoid.urdf` to include the camera:

```xml
  <!-- Camera link -->
  <joint name="head_to_camera" type="fixed">
    <parent link="head"/>
    <child link="camera_frame"/>
    <origin xyz="0.05 0 0" rpy="0 0 0"/>
  </joint>

  <link name="camera_frame">
    <visual>
      <geometry>
        <box size="0.02 0.04 0.02"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    
    <collision>
      <geometry>
        <box size="0.02 0.04 0.02"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>
```

## Adding a LiDAR Sensor

Now, let's add a LiDAR sensor to our robot. We'll mount it on the torso. Add this to `my_first_robot/urdf/humanoid.gazebo.xacro`:

```xml
  <!-- Gazebo LiDAR Sensor Plugin -->
  <gazebo reference="torso">
    <sensor name="lidar" type="ray">
      <always_on>true</always_on>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1.0</resolution>
            <min_angle>-3.14159</min_angle>  <!-- -π -->
            <max_angle>3.14159</max_angle>    <!-- π -->
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>10.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>/</namespace>
          <remapping>~/out:=/lidar/scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
      </plugin>
    </sensor>
  </gazebo>
```

Also add a LiDAR link to the URDF. Add this to `my_first_robot/urdf/humanoid.urdf`:

```xml
  <!-- LiDAR link -->
  <joint name="torso_to_lidar" type="fixed">
    <parent link="torso"/>
    <child link="lidar_frame"/>
    <origin xyz="0.1 0 0.5" rpy="0 0 0"/>  <!-- Mount on front of torso -->
  </joint>

  <link name="lidar_frame">
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.04"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    
    <collision>
      <geometry>
        <cylinder radius="0.02" length="0.04"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="0.05"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0002"/>
    </inertial>
  </link>
```

## Creating a Sensor Data Publisher

Create `my_first_robot/sensor_publisher.py` to simulate publishing sensor data:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion
import numpy as np
from math import pi
import random


class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        
        # Publishers for sensor data
        self.camera_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.lidar_pub = self.create_publisher(LaserScan, '/lidar/scan', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        
        # Timer for publishing sensor data
        self.timer = self.create_timer(0.1, self.publish_sensor_data)  # 10Hz
        
        # For camera image simulation
        self.image_count = 0
        
        # For IMU simulation
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def publish_sensor_data(self):
        # Publish camera data
        self.publish_camera_data()
        
        # Publish LiDAR data
        self.publish_lidar_data()
        
        # Publish IMU data
        self.publish_imu_data()

    def publish_camera_data(self):
        # Create a simple simulated image
        height = 480
        width = 640
        channels = 3  # RGB
        
        # Create a simple gradient image
        image_data = np.zeros((height, width, channels), dtype=np.uint8)
        
        # Add some simulated features for interest
        for y in range(height):
            for x in range(width):
                # Create a color gradient with some "objects"
                r = min(255, int((x / width) * 255))
                g = min(255, int((y / height) * 255))
                b = 100  # Constant blue component
                
                # Add some random "objects"
                if (x - width//3)**2 + (y - height//3)**2 < 30**2:
                    r, g, b = 255, 0, 0  # Red circle
                elif (x - 2*width//3)**2 + (y - 2*height//3)**2 < 25**2:
                    r, g, b = 0, 255, 0  # Green circle
                
                image_data[y, x] = [r, g, b]
        
        # Create ROS Image message
        img_msg = Image()
        img_msg.header = Header()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_frame'
        img_msg.height = height
        img_msg.width = width
        img_msg.encoding = 'rgb8'
        img_msg.is_bigendian = False
        img_msg.step = width * channels
        img_msg.data = image_data.tobytes()
        
        self.camera_pub.publish(img_msg)

    def publish_lidar_data(self):
        # Create a LaserScan message
        scan = LaserScan()
        scan.header = Header()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'lidar_frame'
        
        # Set laser scan parameters
        scan.angle_min = -pi
        scan.angle_max = pi
        scan.angle_increment = 2 * pi / 360  # 360 points
        scan.time_increment = 0.0
        scan.scan_time = 0.1  # Time between scans (10Hz)
        scan.range_min = 0.1
        scan.range_max = 10.0
        
        # Generate range data 
        # Simulate a room with some obstacles
        ranges = []
        for i in range(360):
            angle = scan.angle_min + i * scan.angle_increment
            
            # Simulate a square room with side length 8m, centered at origin
            # With some obstacles represented as shorter distances
            distance = 5.0  # Default distance
            
            # Add some obstacles
            if 80 < i < 100 or 170 < i < 190:
                distance = 2.0
            elif 270 < i < 290:
                distance = 1.5
            elif 350 < i or i < 10:
                distance = 3.0
                
            # Add some randomness for realism
            distance += random.uniform(-0.1, 0.1)
            
            ranges.append(distance)
        
        scan.ranges = ranges
        scan.intensities = []  # No intensity data
        
        self.lidar_pub.publish(scan)

    def publish_imu_data(self):
        # Create an IMU message
        imu = Imu()
        imu.header = Header()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'head'
        
        # For simulation, we'll create somewhat realistic IMU data
        # In a real robot, this would come from the actual IMU sensor
        imu.orientation = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)
        
        # Add some noise to simulate real sensor
        imu.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        
        # Simulate angular velocity (currently 0 for stable robot)
        imu.angular_velocity.x = random.gauss(0.0, 0.01)
        imu.angular_velocity.y = random.gauss(0.0, 0.01)
        imu.angular_velocity.z = random.gauss(0.0, 0.01)
        
        # Simulate linear acceleration (gravity + movement)
        imu.linear_acceleration.x = random.gauss(0.0, 0.1)
        imu.linear_acceleration.y = random.gauss(0.0, 0.1)
        imu.linear_acceleration.z = 9.8 + random.gauss(0.0, 0.1)
        
        # Set covariance
        imu.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        imu.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        
        self.imu_pub.publish(imu)

    def euler_to_quaternion(self, roll, pitch, yaw):
        # Convert Euler angles to quaternion
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        quaternion = Quaternion()
        quaternion.x = qx
        quaternion.y = qy
        quaternion.z = qz
        quaternion.w = qw
        
        return quaternion


def main():
    rclpy.init()
    sensor_publisher = SensorPublisher()
    
    try:
        rclpy.spin(sensor_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Visualizing Sensor Data in RViz

Create an RViz configuration file to visualize sensor data: `my_first_robot/rviz/sensors.rviz`:

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
    - Class: rviz_default_plugins/Camera
      Name: Camera
      Topic: /camera/image_raw
      Image Topic: /camera/image_raw
      Transport Hint: raw
    - Class: rviz_default_plugins/LaserScan
      Name: LaserScan
      Topic: /lidar/scan
      Size (m): 0.05
      Color:
        r: 1
        g: 0.5
        b: 0
    - Class: rviz_default_plugins/Imu
      Name: IMU
      Topic: /imu
      Queue Size: 10
  Global Options:
    Fixed Frame: torso
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Name: Orbit
      Target Frame: torso
      Value: Orbit (rviz)
      Distance: 3.0
      Pitch: 0.3
      Yaw: 0.5
```

## Creating a Sensor Visualization Launch File

Create `my_first_robot/launch/sensor_viz.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Sensor publisher node
        Node(
            package='my_first_robot',
            executable='sensor_publisher',
            name='sensor_publisher',
            output='screen',
        ),
        
        # RViz node with sensor configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('my_first_robot'),
                'rviz',
                'sensors.rviz'
            ])],
            output='screen',
        ),
    ])
```

## Testing the Sensors

First, update your package.xml to include necessary dependencies:

```xml
<depend>sensor_msgs</depend>
<depend>geometry_msgs</depend>
<depend>cv2</depend>
<depend>numpy</depend>
```

Build your workspace:

```bash
cd ~/physical_ai_ws
colcon build --packages-select my_first_robot
source install/setup.bash
```

Test the sensor publishers:

```bash
# Terminal 1: Launch sensor publishers
ros2 run my_first_robot sensor_publisher

# Terminal 2: Echo camera data
ros2 topic echo /camera/image_raw

# Terminal 3: Echo LiDAR data
ros2 topic echo /lidar/scan

# Terminal 4: Echo IMU data
ros2 topic echo /imu
```

Visualize the sensor data in RViz:

```bash
ros2 launch my_first_robot sensor_viz.launch.py
```

## Working with Gazebo Sensors

If you want to use Gazebo's built-in sensors instead of our simulated publisher, you'll need to update your Gazebo launch file to include the proper URDF with sensors. Update `my_first_robot/urdf/humanoid_gazebo.urdf.xacro` to include the camera and LiDAR:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot">
  <!-- Include the base humanoid model -->
  <xacro:include filename="humanoid.urdf"/>

  <!-- Include Gazebo-specific elements and sensors -->
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

## Quiz

1. What message type is used for camera data in ROS 2?
2. What is the purpose of the IMU sensor in robotics?
3. How many points does our simulated LiDAR generate per scan?

## Summary

In this lesson, you've equipped your humanoid robot with essential sensors: a camera for vision, a LiDAR for 3D mapping, and an IMU for orientation sensing. You've learned how to publish realistic sensor data and visualize it in RViz. These sensors enable your robot to perceive and interact with its environment.

In the next lesson, we'll explore how to visualize your robot in the beautiful Unity engine!