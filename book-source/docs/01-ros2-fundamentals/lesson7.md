---
sidebar_position: 7
sidebar_label: 'Lesson 7: Bridging Python AI Agents to ROS Controllers'
slug: bridging-ai-ros-controllers
---

# Bridging Python AI Agents to ROS Controllers

In the exciting world of robotics, artificial intelligence (AI) agents often act as the 'brain' that decides what a robot should do. ROS 2 provides the 'nervous system' that carries out these decisions, allowing different parts of the robot and its control software to communicate effectively. This lesson explores how a Python-based AI agent can conceptually interact with and control a robot managed by ROS 2.

## Conceptual Flow: AI Agent to ROS 2 Robot

Imagine an AI agent designed to navigate a robot through a room. The agent needs to:

1.  **Perceive the environment**: Receive sensor data (e.g., camera images, lidar scans) from the robot.
2.  **Make decisions**: Process this data to determine the next action (e.g., move forward, turn left).
3.  **Command the robot**: Send control commands to the robot's motors or actuators.

ROS 2 provides the perfect framework for this interaction, acting as the communication middleware between your AI agent and the robot's hardware controllers.

### The Role of `rclpy`

`rclpy` is the official Python client library for ROS 2. It provides the necessary tools and APIs to write ROS 2 nodes in Python, enabling your AI agent to seamlessly integrate with the ROS 2 ecosystem. Through `rclpy`, your AI agent can:

*   **Publish commands to topics**: Send messages to ROS 2 topics, which are then received by other nodes (e.g., motor controllers).

![AI Agent Publishing Commands to ROS 2 Topics](/img/01-ros2-fundamentals/ai_ros2_pub.png)
*   **Subscribe to sensor data**: Receive messages from ROS 2 topics, allowing the AI agent to get real-time feedback from the robot's sensors.

![AI Agent Subscribing to ROS 2 Topics for Sensor Feedback](/img/01-ros2-fundamentals/ai_ros2_sub.png)

This makes `rclpy` the crucial bridge, allowing your Python AI code to 'speak' the language of ROS 2 and interact with all the robot's components.

## Simplified AI-ROS 2 Interaction Loop

Here's a runnable example demonstrating a typical interaction loop for a Python AI agent controlling a ROS 2 robot. For this code to run, ensure you have ROS 2 installed and the `std_msgs` and `sensor_msgs` packages available (usually installed with ROS 2). You can simulate sensor data publishing and command subscribing using `ros2 topic pub` and `ros2 topic echo` respectively.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Example message type for commands
from sensor_msgs.msg import Image # Example message type for sensor feedback (replace with actual if needed)
import time

class AIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent_node')
        self.command_publisher = self.create_publisher(String, 'robot_command', 10)
        self.sensor_subscriber = self.create_subscription(
                Image, 'camera_feed', self.sensor_callback, 10)
        self.timer = self.create_timer(1.0, self.main_loop) # Run AI logic every 1 second

        self.latest_sensor_data = None
        self.get_logger().info('AI Agent Node started. Waiting for sensor data...')

    def sensor_callback(self, msg):
        self.latest_sensor_data = msg
        self.get_logger().info(f'Received sensor data (timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec})')

    def main_loop(self):
        if self.latest_sensor_data is not None:
            # 1. AI Perceives: Process sensor data
            ai_perception = self.process_sensor_data(self.latest_sensor_data)
            self.get_logger().info(f'AI Perception: {ai_perception}')

            # 2. AI Decides: Determine command based on perception
            command = self.decide_action(ai_perception)
            self.get_logger().info(f'AI Decision: {command}')

            # 3. AI Commands: Publish command to ROS 2 topic
            command_msg = String()
            command_msg.data = command
            self.command_publisher.publish(command_msg)
            self.get_logger().info(f'Published command: "{command}"')
        else:
            self.get_logger().info('No sensor data yet. Waiting...')

    def process_sensor_data(self, sensor_data):
        # In a real scenario, this would involve complex AI/ML models
        # For this runnable example, we'll just acknowledge receipt.
        return f'Processed data from sensor at {sensor_data.header.stamp.sec}'

    def decide_action(self, ai_perception):
        # In a real scenario, this would involve complex AI/ML decision-making
        # For this runnable example, we'll send a dummy command.
        if 'Processed data' in ai_perception:
            return 'move_forward'
        return 'stand_by'

def main(args=None):
    rclpy.init(args=args)
    ai_agent_node = AIAgentNode()
    rclpy.spin(ai_agent_node)
    ai_agent_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This runnable example illustrates the basic structure:

1.  **Initialization**: The AI agent creates a ROS 2 node using `rclpy`, sets up publishers for commands (`robot_command`) and subscribers for sensor feedback (`camera_feed`).
2.  **Sensor Callback**: This function stores the latest sensor data received.
3.  **Main Loop**: Executed periodically, this loop:
    *   Processes the latest sensor data (AI Perception).
    *   Makes decisions based on the perceived environment (AI Decision).
    *   Publishes commands back to ROS 2 topics, controlling the robot (AI Command).

To run this example:

1.  Save the code as `ai_agent_node.py` in your ROS 2 package (e.g., `my_robot_ws/src/my_ai_package/my_ai_package/ai_agent_node.py`).
2.  Make it executable: `chmod +x ai_agent_node.py`
3.  Run the node: `ros2 run my_ai_package ai_agent_node`
4.  In a separate terminal, simulate sensor data (e.g., publishing a dummy image message):
    `ros2 topic pub /camera_feed sensor_msgs/msg/Image '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'camera_frame'}, height: 100, width: 100, encoding: 'rgb8', is_bigendian: 0, step: 300, data: [0, 0, 0]}' -1`
    (Adjust the message content as needed. You may need to install `ros-<DISTRO>-sensor-msgs` and `ros-<DISTRO>-std-msgs` packages if you haven't already.)
5.  Observe the `ai_agent_node` output and `ros2 topic echo /robot_command` for the published commands.

This example provides a concrete foundation for understanding how `rclpy` enables a Python AI agent to seamlessly become an active participant in a ROS 2 robotic system.