---
sidebar_position: 2
---

# Lesson 2: GPT Turns Words into Robot Actions

In this lesson, we'll create the bridge between natural language commands and robot actions. We'll define how GPT translates user commands into executable robot behaviors using ROS 2 action servers and services.

## Prerequisites
- Completed Module 4, Lesson 1
- Understanding of ROS 2 action servers and services
- OpenAI API access
- Working voice recognition pipeline

## What You'll Build
By the end of this lesson, you'll have:
1. Created a semantic parser that converts natural language to robot actions
2. Implemented action servers for key robot behaviors
3. Built a command execution pipeline that handles GPT-generated actions
4. Tested the complete pipeline from voice to action

## Understanding the Command-to-Action Pipeline

The process flow from voice command to robot action is:

1. User says "Robot, bring me the red cup"
2. Voice recognition converts speech to text
3. GPT parses the text and identifies intent ("fetch_object") and parameters ("red cup")
4. Action planner converts intent to specific ROS 2 action calls
5. Robot executes the sequence of actions

## Creating Action Definitions

First, let's define custom action files for our robot. Create `FetchObject.action`:

```
# FetchObject.action
string object_name
string color
---
bool success
string message
---
float32 progress
string status
```

And `NavigateToObject.action`:

```
# NavigateToObject.action
float64 target_x
float64 target_y
float64 target_z
---
bool success
string message
---
float32 distance_remaining
```

## Implementing Action Servers

Create the action server that implements object fetching:

```python
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from your_robot_interfaces.action import FetchObject, NavigateToObject
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import time

class FetchObjectActionServer(Node):
    def __init__(self):
        super().__init__('fetch_object_action_server')
        self._action_server = ActionServer(
            self,
            FetchObject,
            'fetch_object',
            self.execute_callback,
            cancel_callback=self.cancel_callback)
    
    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
        
    async def execute_callback(self, goal_handle):
        """Execute the action"""
        self.get_logger().info(f'Executing goal: fetch {goal_handle.request.color} {goal_handle.request.object_name}')
        
        # Feedback and result
        feedback_msg = FetchObject.Feedback()
        result = FetchObject.Result()
        
        # Step 1: Find the object using perception system
        feedback_msg.status = f"Searching for {goal_handle.request.color} {goal_handle.request.object_name}"
        goal_handle.publish_feedback(feedback_msg)
        
        # Simulate object detection
        detected_position = self.detect_object(goal_handle.request.object_name, goal_handle.request.color)
        
        if detected_position is None:
            result.success = False
            result.message = f"Could not find {goal_handle.request.color} {goal_handle.request.object_name}"
            goal_handle.succeed()
            return result
        
        # Step 2: Navigate to the object
        feedback_msg.status = f"Navigating to {goal_handle.request.color} {goal_handle.request.object_name}"
        goal_handle.publish_feedback(feedback_msg)
        
        navigate_success = await self.navigate_to_position(detected_position)
        
        if not navigate_success:
            result.success = False
            result.message = "Failed to navigate to object"
            goal_handle.succeed()
            return result
        
        # Step 3: Pick up the object
        feedback_msg.status = f"Attempting to pick up {goal_handle.request.object_name}"
        goal_handle.publish_feedback(feedback_msg)
        
        pickup_success = self.pick_up_object(detected_position)
        
        if not pickup_success:
            result.success = False
            result.message = "Failed to pick up object"
            goal_handle.succeed()
            return result
        
        # Step 4: Return to user
        feedback_msg.status = "Returning to user"
        goal_handle.publish_feedback(feedback_msg)
        
        return_home_success = self.return_to_user_position()
        
        if not return_home_success:
            result.success = False
            result.message = "Failed to return to user"
            goal_handle.succeed()
            return result
        
        # Success!
        result.success = True
        result.message = f"Successfully fetched {goal_handle.request.color} {goal_handle.request.object_name}"
        goal_handle.succeed()
        return result
    
    def detect_object(self, object_name, color):
        """Detect object in the environment (simplified implementation)"""
        # In a real system, this would use computer vision
        # For simulation, we'll return a fixed position
        return {'x': 1.5, 'y': 2.0, 'z': 0.0}
    
    async def navigate_to_position(self, position):
        """Navigate to the specified position using Nav2"""
        # In a real implementation, this would call the Nav2 navigation action
        # Simulate navigation with a delay
        time.sleep(2)
        return True
    
    def pick_up_object(self, position):
        """Pick up object at the given position"""
        # In a real implementation, this would control the robot's arms
        # Simulate picking up with a delay
        time.sleep(1)
        return True
    
    def return_to_user_position(self):
        """Return to the user's position"""
        # In a real implementation, this would navigate back to the user
        # Simulate returning with a delay
        time.sleep(2)
        return True

def main(args=None):
    rclpy.init(args=args)
    fetch_object_action_server = FetchObjectActionServer()
    
    try:
        rclpy.spin(fetch_object_action_server)
    except KeyboardInterrupt:
        pass
    finally:
        fetch_object_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating the Command Execution Pipeline

Create a node that receives GPT-generated commands and executes them:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from your_robot_interfaces.action import FetchObject
from rclpy.action import ActionClient
import json

class CommandExecutorNode(Node):
    def __init__(self):
        super().__init__('command_executor_node')
        
        # Subscribe to voice commands
        self.subscription = self.create_subscription(
            String,
            'voice_commands',
            self.command_callback,
            10)
        
        # Action client for fetching objects
        self.fetch_action_client = ActionClient(
            self,
            FetchObject,
            'fetch_object')
        
        # Ensure action server is available
        self.fetch_action_client.wait_for_server()
        
    def command_callback(self, msg):
        """Process incoming command messages"""
        try:
            # Parse the command (in JSON format from GPT)
            command_data = json.loads(msg.data)
            
            # Execute based on command type
            if command_data['action'] == 'fetch_object':
                self.execute_fetch_command(command_data)
            elif command_data['action'] == 'navigate_to':
                self.execute_navigate_command(command_data)
            # Add more action types as needed
            
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON command: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error executing command: {e}")
    
    def execute_fetch_command(self, command_data):
        """Execute a fetch object command"""
        goal_msg = FetchObject.Goal()
        goal_msg.object_name = command_data.get('params', {}).get('object_type', '')
        goal_msg.color = command_data.get('params', {}).get('color', '')
        
        # Send goal to action server
        self.fetch_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.fetch_feedback_callback)
    
    def fetch_feedback_callback(self, feedback_msg):
        """Handle feedback from the fetch action"""
        self.get_logger().info(f"Fetch feedback: {feedback_msg.status}")
    
    def execute_navigate_command(self, command_data):
        """Execute a navigation command"""
        # Implementation would be similar to fetch but for navigation
        pass

def main(args=None):
    rclpy.init(args=args)
    command_executor_node = CommandExecutorNode()
    
    try:
        rclpy.spin(command_executor_node)
    except KeyboardInterrupt:
        pass
    finally:
        command_executor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Natural Language Understanding with GPT

To improve the natural language understanding, we can create specific prompts for GPT that generate properly formatted actions:

```python
def generate_robot_action(command_text):
    """Generate structured robot action from natural language command"""
    
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
      Params: object_type (string), color (string), location (optional)
    - "navigate_to": To move to a location
      Params: x (float), y (float), z (float)
    - "follow_person": To follow someone
      Params: target (string)
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
            temperature=0  # Lower temperature for more consistent output
        )
        
        # Extract the JSON response
        action_json = response.choices[0].message['content'].strip()
        
        # Clean up the response if it contains markdown code block markers
        if action_json.startswith("```json"):
            action_json = action_json[7:]  # Remove "```json"
        if action_json.endswith("```"):
            action_json = action_json[:-3]  # Remove "```"
            
        return json.loads(action_json)
        
    except Exception as e:
        print(f"Error generating robot action: {e}")
        return None
```

## Testing the Complete Pipeline

To test the end-to-end pipeline:

1. Start your robot simulation (Isaac Sim or Gazebo)
2. Launch the navigation stack (Navigation2)
3. Run the action servers
4. Start the voice command processor
5. Give a command like "Robot, bring me the red cup"

## Quiz

1. What are the key components of the command-to-action pipeline?
2. Why is it important to use action servers instead of simple topics for robot commands?
3. How does the system handle command failures?

## Summary

In this lesson, you created a complete pipeline to convert natural language commands into robot actions. You implemented action servers for key behaviors and created an execution system that translates GPT-structured commands into actual robot behavior. In the next lesson, we'll put everything together for the final project!