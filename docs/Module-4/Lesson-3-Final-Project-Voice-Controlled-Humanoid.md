---
sidebar_position: 3
---

# Lesson 3: Final Project - Full Autonomous Humanoid with Voice Control

In this culminating lesson, we'll integrate all components to create a fully autonomous humanoid robot that responds to voice commands. This project combines everything learned in the previous modules.

## Prerequisites
- Completed all previous modules and lessons
- Working Isaac Sim environment
- Trained perception models
- VSLAM and Navigation2 systems
- Voice recognition and NLP pipeline

## What You'll Build
By the end of this lesson, you'll have:
1. Integrated all systems into a cohesive humanoid robot
2. Demonstrated complete voice-to-action functionality
3. Executed a complex demonstration scenario
4. Documented your final project for presentation

## System Architecture Overview

Our complete autonomous humanoid system consists of:

```
Voice Command → Whisper → GPT → Action Planner → ROS 2 → Robot Hardware/Simulation
                    ↕         ↕         ↕          ↕         ↕
                STT Model   NLP      Action      ROS 2    Isaac Sim/
                            Engine    Server     Network   Gazebo
```

## Integration Pipeline

First, let's create the main system orchestrator:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from your_robot_interfaces.action import FetchObject
from your_robot_interfaces.action import NavigateToObject
from your_robot_interfaces.srv import FindObject
import threading
import queue
import json
import openai
import speech_recognition as sr

class AutonomousHumanoidController(Node):
    def __init__(self):
        super().__init__('autonomous_humanoid_controller')
        
        # Initialize components
        self.voice_command_queue = queue.Queue()
        self.voice_recognizer = sr.Recognizer()
        self.voice_microphone = sr.Microphone()
        
        # ROS 2 publishers/subscribers
        self.voice_sub = self.create_subscription(
            String, 'voice_commands', self.voice_command_callback, 10)
        
        self.status_pub = self.create_publisher(String, 'system_status', 10)
        
        # Action clients
        self.fetch_action_client = ActionClient(
            self, FetchObject, 'fetch_object')
        
        self.nav_action_client = ActionClient(
            self, NavigateToObject, 'navigate_to_object')
        
        # Service clients
        self.find_object_client = self.create_client(
            FindObject, 'find_object')
        
        # Wait for services/actions to be available
        self.fetch_action_client.wait_for_server()
        self.nav_action_client.wait_for_server()
        while not self.find_object_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for find_object service...')
        
        # Start voice processing thread
        self.voice_thread = threading.Thread(target=self.process_voice_commands)
        self.voice_thread.daemon = True
        self.voice_thread.start()
        
        self.get_logger().info("Autonomous Humanoid Controller initialized")
    
    def voice_command_callback(self, msg):
        """Handle incoming voice commands"""
        try:
            command_data = json.loads(msg.data)
            self.execute_command(command_data)
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON command: {msg.data}")
    
    def execute_command(self, command_data):
        """Execute the structured command"""
        action_type = command_data.get('action', '')
        
        if action_type == 'fetch_object':
            self.execute_fetch_command(command_data)
        elif action_type == 'navigate_to':
            self.execute_navigate_command(command_data)
        elif action_type == 'find_person':
            self.execute_find_person_command(command_data)
        else:
            self.get_logger().warn(f"Unknown action type: {action_type}")
    
    def execute_fetch_command(self, command_data):
        """Execute object fetching command"""
        params = command_data.get('params', {})
        object_type = params.get('object_type', '')
        color = params.get('color', '')
        
        # Create and send fetch goal
        goal_msg = FetchObject.Goal()
        goal_msg.object_name = object_type
        goal_msg.color = color
        
        self.fetch_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.fetch_feedback_callback)
    
    def execute_navigate_command(self, command_data):
        """Execute navigation command"""
        params = command_data.get('params', {})
        x = params.get('x', 0.0)
        y = params.get('y', 0.0)
        z = params.get('z', 0.0)
        
        # Create and send navigation goal
        goal_msg = NavigateToObject.Goal()
        goal_msg.target_x = float(x)
        goal_msg.target_y = float(y)
        goal_msg.target_z = float(z)
        
        self.nav_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback)
    
    def fetch_feedback_callback(self, feedback_msg):
        """Handle fetch action feedback"""
        status_msg = String()
        status_msg.data = f"Fetch status: {feedback_msg.status}"
        self.status_pub.publish(status_msg)
    
    def nav_feedback_callback(self, feedback_msg):
        """Handle navigation action feedback"""
        status_msg = String()
        status_msg.data = f"Navigation: {feedback_msg.distance_remaining:.2f}m remaining"
        self.status_pub.publish(status_msg)
    
    def process_voice_commands(self):
        """Process voice commands in a separate thread"""
        with self.voice_microphone as source:
            self.voice_recognizer.adjust_for_ambient_noise(source)
        
        while True:
            try:
                with self.voice_microphone as source:
                    self.get_logger().info("Listening for voice command...")
                    audio = self.voice_recognizer.listen(source, timeout=5, phrase_time_limit=5)
                
                # Convert speech to text using Whisper API
                with open("temp_audio.wav", "wb") as f:
                    f.write(audio.get_wav_data())
                
                with open("temp_audio.wav", "rb") as audio_file:
                    transcript = openai.Audio.transcribe("whisper-1", audio_file)
                
                command_text = transcript.text.strip()
                self.get_logger().info(f"Heard: {command_text}")
                
                # Process with GPT
                action_data = self.generate_robot_action(command_text)
                if action_data:
                    # Publish the structured command
                    cmd_msg = String()
                    cmd_msg.data = json.dumps(action_data)
                    self.voice_sub.publish(cmd_msg)
                
            except sr.WaitTimeoutError:
                continue
            except Exception as e:
                self.get_logger().error(f"Error in voice processing: {e}")
    
    def generate_robot_action(self, command_text):
        """Generate structured robot action from natural language"""
        system_prompt = """
        You are a command translator for a humanoid robot. 
        Convert natural language commands to structured JSON with the following format:
        
        {
          "action": "action_type",
          "params": {
            "param1": "value1",
            "param2": "value2"
          }
        }
        
        Available actions:
        - "fetch_object": To get an object from the environment
          Params: object_type (string), color (string)
        - "navigate_to": To move to a location
          Params: x (float), y (float), z (float)
        - "find_person": To locate a person
          Params: person_name (string)
        
        Always respond with valid JSON only. No explanations.
        """
        
        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": command_text}
                ],
                temperature=0
            )
            
            action_json = response.choices[0].message['content'].strip()
            
            # Clean up potential code block markers
            if action_json.startswith("```json"):
                action_json = action_json[7:]
            if action_json.endswith("```"):
                action_json = action_json[:-3]
                
            return json.loads(action_json)
            
        except Exception as e:
            self.get_logger().error(f"Error with GPT: {e}")
            return None

def main(args=None):
    rclpy.init(args=args)
    controller = AutonomousHumanoidController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Demonstration Scenario

Let's create a complete demonstration scenario where we ask the robot to perform a complex task:

1. Say "Robot, please bring me the red cup from the kitchen"
2. The robot will:
   - Use Whisper to convert speech to text
   - Use GPT to parse "fetch red cup from kitchen"
   - Use VSLAM to map the kitchen area
   - Use Navigation2 to move to the kitchen
   - Use perception to locate the red cup
   - Navigate to the cup and pick it up
   - Return to the user and hand over the cup

## Launch File for Complete System

Create a launch file that starts all required components:

```python
# complete_humanoid_system.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation clock if true'
        ),
        
        # Include Isaac Sim/Gazebo launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('your_robot_gazebo'),
                    'launch',
                    'robot_world.launch.py'
                ])
            ]),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        
        # Include Navigation2 launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': 'True'
            }.items()
        ),
        
        # Launch VSLAM (ORB-SLAM3 or similar)
        Node(
            package='orb_slam3_ros2',
            executable='orb_slam3_ros2',
            name='orb_slam3',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('your_robot_slam'),
                    'config',
                    'orb_slam3_config.yaml'
                ])
            ]
        ),
        
        # Launch action servers
        Node(
            package='your_robot_control',
            executable='fetch_object_server',
            name='fetch_object_server'
        ),
        
        Node(
            package='your_robot_control',
            executable='navigate_to_object_server',
            name='navigate_to_object_server'
        ),
        
        # Launch the main controller
        Node(
            package='your_robot_control',
            executable='autonomous_humanoid_controller',
            name='autonomous_humanoid_controller',
            parameters=[
                {'use_sim_time': use_sim_time}
            ]
        ),
        
        # Launch RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d', 
                PathJoinSubstitution([
                    FindPackageShare('your_robot_control'),
                    'rviz',
                    'humanoid_control.rviz'
                ])
            ]
        )
    ])
```

## Testing the Complete System

To test your complete autonomous humanoid:

1. Launch the complete system:
   ```bash
   ros2 launch your_robot_control complete_humanoid_system.launch.py
   ```

2. Say a command like "Robot, bring me the red cup"

3. Observe as the robot:
   - Recognizes your voice command
   - Plans the appropriate actions
   - Navigates to the object
   - Manipulates the object
   - Returns to you

## Project Documentation

Document your final project with:
1. System architecture diagrams
2. Performance metrics (response time, success rate)
3. Challenges encountered and solutions
4. Future improvements

## Quiz

1. What are the main components of the complete autonomous humanoid system?
2. How does the system handle complex multi-step commands?
3. What safety measures should be implemented in an autonomous robot?

## Summary

Congratulations! You've created a complete autonomous humanoid robot that responds to natural voice commands. You've integrated perception, navigation, manipulation, and natural language processing into a cohesive system. This project demonstrates the convergence of all technologies covered in this course. The next lesson will discuss deployment and production considerations for your robot system.