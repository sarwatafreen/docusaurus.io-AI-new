---
sidebar_position: 3
---

# Lesson 3: Talk to Robot Parts (Topics & Services)

In this lesson, we'll dive deeper into ROS 2 communication patterns. We've already seen topics (used for continuous communication like sensor data and velocity commands), and now we'll explore services (used for request-response communication) and other communication patterns.

## Prerequisites
- Completed Lesson 1: Your First Robot "Hello World"
- Completed Lesson 2: Move the Robot with Keyboard
- Understanding of ROS 2 topics and message publishing

## What You'll Learn
By the end of this lesson, you'll understand:
1. The differences between topics and services in ROS 2
2. How to create and use ROS 2 services
3. Advanced communication patterns like actions
4. How to debug communication between robot parts

## Topics vs Services vs Actions

ROS 2 offers three main communication patterns:

- **Topics**: Used for continuous, asynchronous communication (e.g., sensor data, velocity commands)
- **Services**: Used for request-response communication (e.g., "calibrate sensor", "get robot status")
- **Actions**: Used for long-running tasks with feedback (e.g., "navigate to goal", "pick up object")

## Creating a Simple Service

Let's create a service that returns the robot's status. First, create a `srv` directory:

```bash
mkdir -p my_first_robot/srv
```

Create `my_first_robot/srv/RobotStatus.srv`:

```
# Request
string request_type  # Can be "battery", "location", "status"
---
# Response
string status
float32 value
bool success
```

## Creating a Service Server

Create `my_first_robot/status_service.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_first_robot.srv import RobotStatus


class StatusService(Node):
    def __init__(self):
        super().__init__('status_service')
        
        # Create a service server
        self.srv = self.create_service(
            RobotStatus, 
            'get_robot_status', 
            self.status_callback
        )
        
        # Simulate robot battery level
        self.battery_level = 85.0
        
        # Simulate robot location
        self.location_x = 0.0
        self.location_y = 0.0

    def status_callback(self, request, response):
        self.get_logger().info(f"Received request: {request.request_type}")
        
        if request.request_type == "battery":
            response.status = f"Battery level: {self.battery_level}%"
            response.value = self.battery_level
            response.success = True
        elif request.request_type == "location":
            response.status = f"Location: ({self.location_x:.2f}, {self.location_y:.2f})"
            response.value = 0.0  # Placeholder
            response.success = True
        elif request.request_type == "status":
            response.status = "Robot is operational"
            response.value = 1.0  # 1 for operational, 0 for not operational
            response.success = True
        else:
            response.status = "Unknown request type"
            response.value = -1.0
            response.success = False
            
        return response


def main():
    rclpy.init()
    
    status_service = StatusService()
    
    try:
        rclpy.spin(status_service)
    except KeyboardInterrupt:
        pass
    finally:
        status_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Creating a Service Client

Create `my_first_robot/status_client.py`:

```python
#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from my_first_robot.srv import RobotStatus


class StatusClient(Node):
    def __init__(self):
        super().__init__('status_client')
        self.cli = self.create_client(RobotStatus, 'get_robot_status')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.req = RobotStatus.Request()

    def send_request(self, request_type):
        self.req.request_type = request_type
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()
    
    client = StatusClient()
    
    if len(sys.argv) != 2:
        print("Usage: ros2 run my_first_robot status_client <request_type>")
        print("Request types: battery, location, status")
        sys.exit(1)
    
    request_type = sys.argv[1]
    
    response = client.send_request(request_type)
    if response:
        print(f"Response:")
        print(f"  Status: {response.status}")
        print(f"  Value: {response.value}")
        print(f"  Success: {response.success}")
    else:
        print("Failed to call service")
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Creating an Action Example

Actions are more complex and used for long-running tasks. Create `my_first_robot/move_to_goal.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from my_first_robot.action import MoveToGoal


class MoveToGoalActionServer(Node):
    def __init__(self):
        super().__init__('move_to_goal_action_server')
        
        # Create action server with a reentrant callback group to allow concurrent execution
        self._action_server = ActionServer(
            self,
            MoveToGoal,
            'move_to_goal',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            cancel_callback=self.cancel_callback)

    def destroy_node(self):
        self._action_server.destroy()
        super().destroy_node()

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = MoveToGoal.Feedback()
        result = MoveToGoal.Result()
        
        # Simulate moving to goal (in a real robot, this would control motors)
        for i in range(1, 11):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                result.reached = False
                return result
            
            # Update feedback
            feedback_msg.current_distance = float(i * 10)  # 10%, 20%, etc.
            feedback_msg.remaining_distance = float((10 - i) * 10)
            
            self.get_logger().info(f'Feedback: {feedback_msg.current_distance}% complete')
            goal_handle.publish_feedback(feedback_msg)
            
            # Simulate some work (in reality, this would be the actual movement)
            from time import sleep
            sleep(0.5)
        
        # Goal completed
        goal_handle.succeed()
        result.reached = True
        self.get_logger().info('Goal succeeded')
        
        return result


def main():
    rclpy.init()
    
    action_server = MoveToGoalActionServer()
    
    try:
        executor = MultiThreadedExecutor()
        rclpy.spin(action_server, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Creating the Action Definition

For the action example above, we need to define the action type. Create an `action` directory:

```bash
mkdir -p my_first_robot/action
```

Create `my_first_robot/action/MoveToGoal.action`:

```
# Goal definition
float32 x_goal
float32 y_goal
---
# Result definition
bool reached
---
# Feedback definition
float32 current_distance
float32 remaining_distance
```

## Debugging ROS Communication

ROS 2 provides several tools for debugging communication:

1. **Check active topics**:
   ```bash
   ros2 topic list
   ```

2. **Echo topic data**:
   ```bash
   ros2 topic echo /topic_name
   ```

3. **Check active services**:
   ```bash
   ros2 service list
   ```

4. **Call a service**:
   ```bash
   ros2 service call /service_name service_type "{request_field: value}"
   ```

## Quiz

1. What is the difference between a topic and a service in ROS 2?
2. When would you use an action instead of a topic or service?
3. Name three tools for debugging ROS 2 communication.

## Summary

In this lesson, you've learned about the different communication patterns in ROS 2: topics, services, and actions. Each pattern serves different use cases in robotics. Topics work well for continuous data streams like sensors, services are ideal for request-response interactions like calibration, and actions handle long-running tasks with feedback like navigation.

In the next lesson, we'll learn how to build your own humanoid robot model using URDF!