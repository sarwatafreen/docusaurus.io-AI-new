---
sidebar_position: 4
---

# Lesson 4: See Your Robot in Beautiful Unity

In this lesson, we'll explore Unity as an alternative simulation environment for our humanoid robot. While Gazebo is excellent for physics simulation, Unity offers stunning visuals and realistic rendering that can be valuable for perception training and user interfaces. We'll learn how to export our robot model to Unity and create a visually rich simulation.

## Prerequisites
- Completed Module 1: ROS 2 basics
- Module 2, Lessons 1-3: Gazebo simulation and sensor integration
- Understanding of robot modeling and simulation concepts

## What You'll Build
By the end of this lesson, you'll have:
1. Exported your URDF robot model to a format compatible with Unity
2. Created a Unity project with your humanoid robot
3. Set up basic robot control in Unity
4. Understood how to integrate Unity with ROS 2 using ROS# or similar tools

## Understanding Unity for Robotics

Unity is a powerful game engine that's increasingly being used in robotics for:
- High-quality visualization and rendering
- Perception training with photorealistic environments
- User interface development
- Human-robot interaction design
- Simulation of complex sensor data

## Exporting URDF to Unity-Compatible Format

Unity doesn't directly read URDF files, so we need to convert our robot model. We have a few options:

### Option 1: Using the URDF Importer for Unity

Unity provides a URDF Importer tool that can directly import URDF files:

1. Download the URDF Importer package from the Unity Asset Store or Unity Robotics GitHub
2. Import it into your Unity project
3. Create a ROS-enabled Unity scene

### Option 2: Converting to OBJ/FBX manually

We can manually convert parts of our URDF model by exporting individual geometries as OBJ or FBX files.

Create a simple Python script to export basic geometries: `my_first_robot/scripts/urdf_to_obj.py`

```python
#!/usr/bin/env python3

import xml.etree.ElementTree as ET
import os
from math import pi


def urdf_to_obj(urdf_path, output_dir):
    """
    Extracts geometries from URDF and creates simple OBJ files.
    This is a simplified version for demonstration.
    """
    os.makedirs(output_dir, exist_ok=True)
    
    # Parse URDF file
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    # Process each link
    for link in root.findall('link'):
        link_name = link.get('name')
        visual_element = link.find('visual')
        
        if visual_element is not None:
            geometry_element = visual_element.find('geometry')
            
            if geometry_element is not None:
                # Handle different geometry types
                if geometry_element.find('box') is not None:
                    box = geometry_element.find('box')
                    size = [float(x) for x in box.get('size').split()]
                    create_box_obj(link_name, size, os.path.join(output_dir, f'{link_name}.obj'))
                
                elif geometry_element.find('sphere') is not None:
                    sphere = geometry_element.find('sphere')
                    radius = float(sphere.get('radius'))
                    create_sphere_obj(link_name, radius, os.path.join(output_dir, f'{link_name}.obj'))
                
                elif geometry_element.find('cylinder') is not None:
                    cylinder = geometry_element.find('cylinder')
                    radius = float(cylinder.get('radius'))
                    length = float(cylinder.get('length'))
                    create_cylinder_obj(link_name, radius, length, os.path.join(output_dir, f'{link_name}.obj'))
                
                elif geometry_element.find('capsule') is not None:
                    # Capsules are usually cylinders with hemispherical ends
                    capsule = geometry_element.find('capsule')
                    radius = float(capsule.get('radius'))
                    length = float(capsule.get('length'))
                    create_capsule_obj(link_name, radius, length, os.path.join(output_dir, f'{link_name}.obj'))


def create_box_obj(name, size, output_path):
    """Create an OBJ file for a box geometry."""
    width, depth, height = size
    
    # Define the 8 vertices of a box centered at origin
    vertices = [
        (-width/2, -depth/2, -height/2),  # v0
        (width/2, -depth/2, -height/2),   # v1
        (width/2, depth/2, -height/2),    # v2
        (-width/2, depth/2, -height/2),   # v3
        (-width/2, -depth/2, height/2),   # v4
        (width/2, -depth/2, height/2),    # v5
        (width/2, depth/2, height/2),     # v6
        (-width/2, depth/2, height/2),    # v7
    ]
    
    # Define the 6 faces of the box
    faces = [
        [1, 2, 3, 4],    # Bottom face
        [5, 8, 7, 6],    # Top face
        [1, 4, 8, 5],    # Front face
        [2, 6, 7, 3],    # Back face
        [4, 3, 7, 8],    # Left face
        [1, 5, 6, 2],    # Right face
    ]
    
    with open(output_path, 'w') as f:
        f.write(f"# Box object: {name}\n")
        
        # Write vertices
        for v in vertices:
            f.write(f"v {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}\n")
        
        # Write texture coordinates (optional)
        f.write("vt 0.0 0.0\n")
        f.write("vt 1.0 0.0\n")
        f.write("vt 1.0 1.0\n")
        f.write("vt 0.0 1.0\n")
        
        # Write normals (optional)
        f.write("vn 0.0 0.0 1.0\n")  # Example normal
        
        # Write faces
        for face in faces:
            # OBJ format: f v/vt/vn v/vt/vn v/vt/vn v/vt/vn
            face_str = "f " + " ".join([f"{v}//1" for v in face])
            f.write(face_str + "\n")


def create_sphere_obj(name, radius, output_path):
    """Create an OBJ file for a sphere geometry (simplified)."""
    # This is a very simplified sphere representation
    # In practice, you'd use a more sophisticated method
    
    with open(output_path, 'w') as f:
        f.write(f"# Sphere object: {name}\n")
        
        # Create a simple icosphere approximation
        # Using 12 vertices of an icosahedron for simplicity
        # In reality, this would be much more complex
        
        # For this example, just create a simple sphere with 8 vertices
        vertices = [
            (0, 0, radius),      # Top
            (radius, 0, 0),      # Right
            (0, radius, 0),      # Front
            (-radius, 0, 0),     # Left
            (0, -radius, 0),     # Back
            (0, 0, -radius),     # Bottom
        ]
        
        # Write vertices
        for i, v in enumerate(vertices):
            f.write(f"v {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}\n")


def create_cylinder_obj(name, radius, length, output_path):
    """Create an OBJ file for a cylinder geometry (simplified)."""
    with open(output_path, 'w') as f:
        f.write(f"# Cylinder object: {name}\n")
        
        # Create a simple cylinder with 20 vertices for each circle
        segments = 20
        vertices = []
        
        # Top circle
        for i in range(segments):
            angle = 2 * pi * i / segments
            x = radius * cos(angle)
            y = radius * sin(angle)
            z = length / 2
            vertices.append((x, y, z))
        
        # Bottom circle
        for i in range(segments):
            angle = 2 * pi * i / segments
            x = radius * cos(angle)
            y = radius * sin(angle)
            z = -length / 2
            vertices.append((x, y, z))
        
        # Write vertices
        for v in vertices:
            f.write(f"v {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}\n")


def create_capsule_obj(name, radius, length, output_path):
    """Create an OBJ file for a capsule geometry (simplified)."""
    with open(output_path, 'w') as f:
        f.write(f"# Capsule object: {name}\n")
        
        # Simplified as a cylinder with hemispherical caps
        # This would require more complex geometry in practice


if __name__ == "__main__":
    import sys
    if len(sys.argv) != 3:
        print("Usage: python urdf_to_obj.py <input_urdf> <output_directory>")
        sys.exit(1)
        
    urdf_path = sys.argv[1]
    output_dir = sys.argv[2]
    
    urdf_to_obj(urdf_path, output_dir)
    print(f"Converted URDF at {urdf_path} to OBJ files in {output_dir}")
```

## Setting Up Unity Project

1. Install Unity Hub and create a new 3D project
2. Import the ROS# package from the Unity Asset Store or GitHub
3. Create a new scene for your robot

In Unity, create a simple controller script for your robot: `Assets/Scripts/RobotController.cs`

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;

public class RobotController : MonoBehaviour
{
    ROSConnection ros;
    public string robotName = "humanoid_robot";
    
    // Robot joint transforms
    public Transform torso;
    public Transform head;
    public Transform leftUpperArm;
    public Transform leftLowerArm;
    public Transform rightUpperArm;
    public Transform rightLowerArm;
    public Transform leftUpperLeg;
    public Transform leftLowerLeg;
    public Transform leftFoot;
    public Transform rightUpperLeg;
    public Transform rightLowerLeg;
    public Transform rightFoot;
    
    // Topics to subscribe to
    const string JOINT_STATES_TOPIC = "/joint_states";
    
    // Joint state variables
    Dictionary<string, float> jointPositions = new Dictionary<string, float>();
    
    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterSubscribeListener<JointStateMsg>(JOINT_STATES_TOPIC, JointStateCallback);
    }
    
    void JointStateCallback(JointStateMsg jointState)
    {
        // Update dictionary with current joint positions
        for (int i = 0; i < jointState.name.Array.Length; i++)
        {
            string jointName = jointState.name.Array[i];
            float position = (float)jointState.position[i];
            
            if (!jointPositions.ContainsKey(jointName))
            {
                jointPositions.Add(jointName, position);
            }
            else
            {
                jointPositions[jointName] = position;
            }
        }
    }
    
    // Update is called once per frame
    void Update()
    {
        // Update robot joint positions based on received joint states
        UpdateJointPosition("torso_to_head", head, 0, 0, 0);
        UpdateJointPosition("torso_to_left_shoulder", leftUpperArm, 0, 0, 1);
        UpdateJointPosition("left_elbow", leftLowerArm, 0, 0, 1);
        UpdateJointPosition("torso_to_right_shoulder", rightUpperArm, 0, 0, 1);
        UpdateJointPosition("right_elbow", rightLowerArm, 0, 0, 1);
        UpdateJointPosition("torso_to_left_hip", leftUpperLeg, 0, 0, 1);
        UpdateJointPosition("left_knee", leftLowerLeg, 0, 0, 1);
        UpdateJointPosition("left_ankle", leftFoot, 1, 0, 0);
        UpdateJointPosition("torso_to_right_hip", rightUpperLeg, 0, 0, 1);
        UpdateJointPosition("right_knee", rightLowerLeg, 0, 0, 1);
        UpdateJointPosition("right_ankle", rightFoot, 1, 0, 0);
    }
    
    void UpdateJointPosition(string jointName, Transform jointTransform, float x, float y, float z)
    {
        if (jointPositions.ContainsKey(jointName))
        {
            float position = jointPositions[jointName];
            
            // Apply rotation based on joint position
            Vector3 rotationAxis = new Vector3(x, y, z);
            jointTransform.Rotate(rotationAxis, position * Mathf.Rad2Deg, Space.Self);
        }
    }
}
```

## Creating Unity ROS Bridge

For Unity to communicate with ROS 2, we need to set up a bridge. Create a ROS bridge node in our package: `my_first_robot/unity_bridge.py`

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time
import numpy as np


class UnityBridgeNode(Node):
    def __init__(self):
        super().__init__('unity_bridge_node')
        
        # Publisher for joint states (Unity will subscribe to this)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Timer for publishing joint states
        self.timer = self.create_timer(0.05, self.publish_joint_states)  # 20Hz
        
        # Simulate some joint positions for demonstration
        self.joint_names = [
            'torso_to_head',
            'torso_to_left_shoulder', 
            'left_elbow',
            'torso_to_right_shoulder',
            'right_elbow',
            'torso_to_left_hip',
            'left_knee',
            'left_ankle',
            'torso_to_right_hip',
            'right_knee',
            'right_ankle'
        ]
        
        self.joint_positions = [0.0] * len(self.joint_names)
        self.time_counter = 0.0

    def publish_joint_states(self):
        # Create joint state message
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'torso'
        
        # Update joint positions with some oscillating values for demo
        self.time_counter += 0.1
        for i in range(len(self.joint_positions)):
            # Apply a different oscillation pattern to each joint
            self.joint_positions[i] = 0.5 * np.sin(self.time_counter + i * 0.5)
        
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = [0.0] * len(self.joint_positions)
        msg.effort = [0.0] * len(self.joint_positions)
        
        self.joint_state_pub.publish(msg)


def main():
    rclpy.init()
    unity_bridge_node = UnityBridgeNode()
    
    try:
        rclpy.spin(unity_bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        unity_bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Launch File for Unity Integration

Create `my_first_robot/launch/unity_integration.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        # Unity bridge node to publish joint states
        Node(
            package='my_first_robot',
            executable='unity_bridge',
            name='unity_bridge_node',
            output='screen',
        ),
        
        # Robot state publisher (for TF and visualization)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': open(os.path.join(
                    get_package_share_directory('my_first_robot'),
                    'urdf',
                    'humanoid.urdf'
                )).read()
            }],
        ),
    ])
```

## Creating a Unity Scene

To create a Unity scene with your robot:

1. Open Unity Hub and create a new 3D project
2. Import ROS# package 
3. Create an empty GameObject and attach the RobotController.cs script
4. Add child GameObjects for each robot part (torso, head, arms, legs) with appropriate transforms
5. Assign these GameObjects to the public fields in the RobotController script
6. Build the Unity project

## Building and Running Unity Integration

1. First, build your ROS 2 package:
```bash
cd ~/physical_ai_ws
colcon build --packages-select my_first_robot
source install/setup.bash
```

2. Launch the ROS 2 side:
```bash
ros2 launch my_first_robot unity_integration.launch.py
```

3. In Unity, build and run your scene with the RobotController script
4. Make sure Unity is configured to connect to your ROS 2 master

## Alternative: Using Isaac Sim

For more advanced Unity-based robotics simulation, NVIDIA Isaac Sim provides:
- Photorealistic rendering 
- High-fidelity physics
- Synthetic data generation
- AI training environments

Isaac Sim is covered in detail in Module 3.

## Quiz

1. What are the main advantages of using Unity for robot simulation?
2. Which tool allows Unity to communicate with ROS 2?
3. What message type is typically used to communicate joint positions between ROS 2 and Unity?

## Summary

In this lesson, you've learned how to take your humanoid robot into the Unity environment for high-quality visualization. You've created scripts to convert URDF to Unity-compatible formats, set up ROS communication with Unity, and created a basic Unity scene with robot control.

This completes Module 2 of our Physical AI & Humanoid Robotics course. You now have experience with both physics-based simulation (Gazebo) and high-quality visualization (Unity). In Module 3, we'll explore NVIDIA Isaac Sim for AI-powered robotics simulation!