---
sidebar_position: 6
title: Streamlining ROS 2 with Launch Files
proficiency_level: A2
---

Managing many ROS 2 nodes can get tricky. You might have several programs working together. Launch files are here to make this much easier. They are like a conductor for your ROS 2 orchestra.

## What Are Launch Files?

Launch files are special XML or Python files. They tell ROS 2 how to start and arrange your applications. Instead of running each node separately, you use one command. This single command then starts everything according to your launch file's instructions.

### Why Use Launch Files?

Launch files bring several big advantages to complex ROS 2 setups:

*   **Orchestrating Multiple Nodes**: Start all needed nodes with just one command. This saves time and reduces errors.
*   **Managing Parameters**: Easily set node parameters in one place. Change behavior without altering code.
*   **Simplifying Startup**: Complex systems become simple to launch. One command does it all.
*   **Reusability**: Use the same launch file across different robots or scenarios. Just adjust parameters.
*   **Debugging**: Quickly identify issues by inspecting the launch process.

## Anatomy of a Launch File

ROS 2 launch files are typically written in Python. This allows for powerful and flexible configurations. Let's look at a basic structure.

### Example Launch File: Python

Here is a simple launch file to start two nodes, a `talker` and a `listener`.

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_pubsub',
            executable='talker',
            name='my_talker',
            parameters=[{'message_to_send': 'Hello ROS 2!'}]
        ),
        Node(
            package='py_pubsub',
            executable='listener',
            name='my_listener'
        ),
    ])
```

Output:
This launch file, when executed, will:
*   Start the `talker` node from the `py_pubsub` package.
*   Assign it the name `my_talker`.
*   Set a parameter `message_to_send` to `Hello ROS 2!`.
*   Start the `listener` node from the `py_pubsub` package.
*   Assign it the name `my_listener`.

### Explaining the Code

Let's break down what's happening in the example:

*   `LaunchDescription`: This is the main container for your launch actions.
*   `Node`: Represents a single ROS 2 executable.
*   `package`: The ROS 2 package where your node resides.
*   `executable`: The name of the executable for the node.
*   `name`: A unique name for this instance of the node.
*   `parameters`: A list of key-value pairs to configure the node.

## Launching Your Application

To run a launch file, you use the `ros2 launch` command:

```bash
ros2 launch my_package my_launch_file.py
```

Output:
This command will execute the specified launch file. It will then start all the nodes and apply all configurations defined within it.

## Exploring Node Parameters

Launch files are excellent for managing node parameters. Imagine a node that controls a robot's speed. You can set the initial speed directly in the launch file.

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_controller',
            executable='speed_controller',
            name='robot_speed_node',
            parameters=[
                {'max_speed': 1.5},
                {'min_speed': 0.1}
            ]
        ),
    ])
```

Output:
This setup starts `speed_controller` and gives it `max_speed` and `min_speed` values. No need to recompile code to change these!

## Try With AI

Let's practice creating a launch file. Design a launch file that:
*   Starts a `camera_publisher` node from the `image_pipeline` package.
*   Starts an `image_viewer` node from the `image_tools` package.
*   Sets the `camera_topic` parameter for `image_viewer` to `/my_robot/camera/image`.

## 🚀 Passing Parameters with Launch Files

Ever wanted to customize your ROS 2 nodes without changing their code? That's where parameters come in handy! Parameters are like adjustable settings for your nodes. You can use them to tell your node things like what topic to subscribe to, how fast to publish, or even a custom message to display.

While you can set parameters directly from the command line, using a launch file is super powerful. It lets you define all your node's settings in one place, making your robot's startup much smoother and easier to manage! Imagine setting up a whole fleet of robots with just one command!

### 🛠️ Setting Up a Node with Parameters

Let's create a simple Python node that says hello, but with a custom message set by a parameter!

First, let's make our Python node. This node will look for a parameter called `custom_message`. If it finds it, it will print that message; otherwise, it will use a default message.

```python
# C:/Users/dell/Desktop/physical-ai/worktrees/chapter1/docs/01-ros2-fundamentals/my_custom_node.py
import rclpy
from rclpy.node import Node

class MyCustomNode(Node):
    def __init__(self):
        super().__init__('my_custom_node')
        self.declare_parameter('custom_message', 'Hello from default!')
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = self.get_parameter('custom_message').get_parameter_value().string_value
        self.get_logger().info('Node says: \"%s\"' % msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyCustomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```
Output:
```

[INFO] [my_custom_node]: Node says: \"Hello from default!\"\n
```

In this code:
*   We `declare_parameter` to tell ROS 2 that our node expects `custom_message`. We even give it a default value!
*   The `timer_callback` retrieves the current value of `custom_message` and prints it.

### 🚀 Launching with Parameters

Now, let's create a Python launch file to run `my_custom_node` and set its `custom_message` parameter.

```python
# C:/Users/dell/Desktop/physical-ai/worktrees/chapter1/docs/01-ros2-fundamentals/my_custom_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='_this_package_name_', # Replace with your actual ROS 2 package name
            executable='my_custom_node',
            name='my_greeting_node',
            parameters=[
                {'custom_message': 'Greetings from the launch file!'}
            ]
        )
    ])

```

Before running this, you'll need to replace `_this_package_name_` with the actual name of your ROS 2 package where `my_custom_node.py` resides. If you haven't created a package yet, for this example, you can place `my_custom_node.py` and `my_custom_launch.py` directly in a directory and run them as standalone Python scripts, but using a package is the ROS 2 way!

### 🏃‍♀️ Try It Out!

To see this in action:

1.  **Save the node:** Save the `my_custom_node.py` code to a file in your workspace, for example, `C:/Users/dell/Desktop/physical-ai/worktrees/chapter1/docs/01-ros2-fundamentals/my_custom_node.py`.
2.  **Save the launch file:** Save the `my_custom_launch.py` code to a file in the same directory, for example, `C:/Users/dell/Desktop/physical-ai/worktrees/chapter1/docs/01-ros2-fundamentals/my_custom_launch.py`.
3.  **Install `launch_ros`**: If you haven't already, install the `launch_ros` package:
    ```bash
    pip install ros-humble-launch-ros # Or your ROS 2 distribution

```
4.  **Run the launch file:**
    ```bash
    ros2 launch C:/Users/dell/Desktop/physical-ai/worktrees/chapter1/docs/01-ros2-fundamentals/my_custom_launch.py

```
Output:
```

[INFO] [my_greeting_node]: Node says: \"Greetings from the launch file!\"\n
```

Notice how the `my_custom_node` now prints the message we provided in the launch file! This is the magic of parameters and launch files working together.

### 🧠 Explore More!

*   What happens if you remove the `parameters` line from the launch file?\n*   How would you add another parameter to `my_custom_node`, for example, to control the `timer_period`?\n*   Can you imagine a scenario where parameters would be super useful for your robot?

## Practice

You have previously developed a publisher/subscriber node and created a `my_robot.launch.py` file to launch them. Let's analyze its structure and functionality.

Consider the following `my_robot.launch.py` file:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_publisher',
            name='robot_publisher',
            parameters=[{'publish_frequency': 10.0}]
        ),
        Node(
            package='my_package',
            executable='my_subscriber',
            name='robot_subscriber'
        ),
    ])
```

Output:
This launch file starts a `my_publisher` node and a `my_subscriber` node from `my_package`. The `robot_publisher` node's `publish_frequency` is set to 10.0 Hz.

Now, answer the following questions about this launch file:

1.  **Launch File Purpose**: What is the primary purpose of `my_robot.launch.py`?
2.  **Nodes Launched**: Which two ROS 2 nodes are launched by this file?
3.  **Package Names**: From which package do these nodes originate?
4.  **Node Renaming**: Are any nodes assigned a custom name different from their executable name? If so, which one(s)?
5.  **Parameter Passing**: Is a parameter being passed to any of the nodes? If yes, which node receives it, what is the parameter name, and what is its value?
6.  **Extensibility**: If you wanted to add a third node, `sensor_reader`, from `another_package` to this launch file, where would you add it and what essential information would you need to provide?

> **💬 AI Colearning Prompt**: Reflect on your answers. Discuss with the AI the benefits of using launch files for managing complex robotic systems. How do they simplify development and deployment?
