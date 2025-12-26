---
sidebar_position: 2
title: 'ROS 2 Communication Model'
---

# ROS 2 Communication Model

## Nodes

In ROS 2, a node is an executable that uses ROS 2 to communicate with other nodes. Nodes are the fundamental building blocks of a ROS 2 system. Each node typically performs a specific task, such as:

- Reading data from a sensor
- Processing data
- Controlling an actuator
- Providing a user interface

### Creating a Node

Nodes are typically created using client libraries like `rclpy` for Python or `rclcpp` for C++. Here's a basic example of a node in Python:

```python
import rclpy
from rclpy.node import Node

class HumanoidControllerNode(Node):
    def __init__(self):
        super().__init__('humanoid_controller')
        self.get_logger().info('Humanoid Controller Node Started')

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics and Publishers/Subscribers

Topics enable asynchronous communication between nodes using a publish/subscribe pattern. A node publishes messages to a topic, and other nodes subscribe to that topic to receive the messages.

### Example: Joint State Communication

For humanoid robots, topics are often used to publish joint states:

```python
from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = JointState()
        msg.name = ['left_hip', 'left_knee', 'right_hip', 'right_knee']
        msg.position = [0.0, 0.0, 0.0, 0.0]
        self.publisher.publish(msg)
```

## Services

Services provide synchronous request/response communication. A client sends a request and waits for a response from a service server.

### Example: Robot Configuration Service

```python
from example_interfaces.srv import SetBool
import rclpy
from rclpy.node import Node

class RobotConfigService(Node):
    def __init__(self):
        super().__init__('robot_config_service')
        self.srv = self.create_service(
            SetBool,
            'set_robot_mode',
            self.set_robot_mode_callback
        )

    def set_robot_mode_callback(self, request, response):
        if request.data:
            self.get_logger().info('Robot set to active mode')
            response.success = True
            response.message = 'Robot mode set to active'
        else:
            self.get_logger().info('Robot set to standby mode')
            response.success = True
            response.message = 'Robot mode set to standby'
        return response
```

## Actions

Actions are used for long-running tasks that may take time to complete and may be cancelable. They provide feedback during execution.

### Example: Navigation Action

```python
from rclpy.action import ActionServer
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose

class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        # Implementation for navigation action
        result = NavigateToPose.Result()
        goal_handle.succeed()
        return result
```

## Agent ↔ Controller Communication Flow

In humanoid robotics, the communication between AI agents and robot controllers typically follows this pattern:

1. **Perception**: Sensors publish data to topics (camera images, IMU data, joint states)
2. **Processing**: AI agents subscribe to sensor data and process it
3. **Decision**: AI agents determine appropriate actions based on processed data
4. **Control**: AI agents send commands via topics or services to robot controllers
5. **Execution**: Robot controllers execute commands and update robot state
6. **Feedback**: Controllers publish updated state information for monitoring

This communication flow enables tight integration between AI decision-making and physical robot control.

## Quality of Service (QoS) Settings

QoS settings are crucial for humanoid robots to ensure reliable communication:

- **Reliability**: Ensure critical control messages are delivered
- **Durability**: Maintain message history for late-joining nodes
- **Deadline**: Ensure messages meet timing constraints
- **Liveliness**: Monitor node availability for safety

### QoS Example for Safety-Critical Communication

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# For safety-critical control messages
qos_profile = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST
)
```

## Best Practices for Humanoid Robotics

1. **Use appropriate QoS settings** for different types of messages (critical vs. non-critical)
2. **Implement proper error handling** for communication failures
3. **Design robust node lifecycle management** to handle node crashes
4. **Consider network topology** when designing communication patterns
5. **Monitor communication performance** to identify bottlenecks

## Summary

This chapter covered the ROS 2 communication model in depth, including:

- How nodes serve as the fundamental building blocks of ROS 2 systems
- The publish/subscribe pattern using topics and publishers/subscribers
- Synchronous communication with services
- Long-running tasks with actions
- The agent ↔ controller communication flow essential for humanoid robotics
- Quality of Service (QoS) settings for reliable communication
- Best practices for humanoid robotics

## Navigation

- **Next**: [Robot Structure with URDF](./robot-structure-urdf.md) - Learn how to describe robot structure using URDF (Unified Robot Description Format) specifically for humanoid robots
- **Previous**: [Introduction to ROS 2 for Physical AI](./introduction-to-ros2.md)