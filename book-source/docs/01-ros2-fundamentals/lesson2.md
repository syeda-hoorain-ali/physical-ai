---
sidebar_position: 2
sidebar_label: 'Lesson: 2 ROS 2 Communication: Topics, Services, and Actions'
slug: ros2-communication
---

# ROS 2 Communication: Topics, Services, and Actions

Robots communicate using different methods. In ROS 2, the main ways are Topics, Services, and Actions. Each one is designed for a specific kind of interaction, making your robot applications flexible and robust.

## Understanding ROS 2 Topics: The Data Freeway 🚀

Imagine robots exchanging information continuously! ROS 2 Topics act like a data freeway, letting different parts of your robot system share real-time data. It's a constant flow of messages.

### The Publish/Subscribe Magic ✨

Topics use a "publish/subscribe" pattern:

*   **Publishers:** Nodes that send messages to a specific Topic.
    *   They "broadcast" data, like a sensor sending temperature.
*   **Subscribers:** Nodes that listen for messages on a Topic.
    *   They "tune in" to receive data, like a display showing temperature.

It's like listening to your favorite radio station! Publishers broadcast, and subscribers listen.

### Why Topics Rule: Awesome Use Cases! 🌟

Topics are perfect for data streams.

*   **Sensor Data:** A robot's camera streams video frames.
    *   A publisher sends each frame to a "camer-images" Topic.
    *   A subscriber processes these frames.
*   **Robot State:** Your robot's position or battery level.
    *   A publisher updates a "robot_status" Topic.
    *   Other nodes subscribe to stay informed.

It's all about keeping everyone in the loop, all the time!

### Code Example: Talking with a Topic 💬

Let's see a simple Python example.

```python title="publisher_node.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'chat_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2! Count: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```python title="subscriber_node.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chat_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

To run these, first source your ROS 2 environment, then run each node in a separate terminal:

:::info[Terminal 1 (Publisher)]
```bash
python publisher_node.py
```

Output:
```text
[INFO] [1678881234.567890]: Publishing: "Hello ROS 2! Count: 0"
[INFO] [1678881235.067890]: Publishing: "Hello ROS 2! Count: 1"
[INFO] [1678881235.567890]: Publishing: "Hello ROS 2! Count: 2"
# ... and so on
```
:::

:::info[Terminal 2 (Subscriber)]
```bash
python subscriber_node.py
```

Output:
```text
[INFO] [1678881234.600000]: I heard: "Hello ROS 2! Count: 0"
[INFO] [1678881235.100000]: I heard: "Hello ROS 2! Count: 1"
[INFO] [1678881235.600000]: I heard: "Hello ROS 2! Count: 2"
# ... and so on
```
:::

The subscriber receives messages published by the publisher! This is core ROS 2 communication.

### Explore Topics with `ros2 topic` 🕵️‍♀️

You can explore active Topics using the ROS 2 CLI!

:::info[List Topics]
See all active Topics in your system.

```bash
ros2 topic list
```

Output:
```text
/chat_topic
/parameter_events
/rosout
```
:::

:::info[Echo Topic]
View messages on a specific Topic in real-time.

```bash
ros2 topic echo /chat_topic
```

Output:
```text
data: Hello ROS 2! Count: 0
---
data: Hello ROS 2! Count: 1
---
data: Hello ROS 2! Count: 2
---
# ... and so on
```
:::

:::info[Topic Info]
Get details about a Topic, like its type and publishers/subscribers.

```bash
ros2 topic info /chat_topic
```
Output:
```text
Type: std_msgs/msg/String
Publisher count: 1
Subscriber count: 1
```
:::

These commands are super handy for debugging and understanding your robot's data flow.

### Try With AI: Your Turn to Chat with ROS 2! 🤖

Now it's your turn to get hands-on!

1.  Create a new ROS 2 Python package (if you haven't already).
2.  Implement a simple publisher node that publishes custom messages.
3.  Implement a corresponding subscriber node to receive and print messages.
4.  Use `ros2 topic list` and `ros2 topic echo` to verify your setup.

Share your code and output, and I'll help you explore further!

## Topic Triumphs: Mastering `ros2 topic` 🚀

Ready to dive deeper into ROS 2 communication? Topics are where the magic happens! They are the main channels for nodes to send and receive messages. Think of them as dedicated chat rooms for your robots!

You'll use the `ros2 topic` CLI commands to peek into these chat rooms, send your own messages, and check how busy they are. Let's explore the essentials!

### Discovering Topics: `ros2 topic list` 📜

Want to know what topics are buzzing around in your ROS 2 system? This command shows all active topics. It's like a directory of communication channels.

:::info[List Topics]
```bash
ros2 topic list
```

Output:
```text
/parameter_events
/rosout
/robot_chatter
/sensor_data
```
:::

This output shows common topics. `/robot_chatter` might be for robot conversations, and `/sensor_data` for readings!

### Eavesdropping on Topics: `ros2 topic echo` 👂

Once you know a topic exists, you might want to listen in. This command prints all messages published on a specific topic. Perfect for seeing the actual data flowing through your robot's nervous system.

:::info[Echo Topic]
```bash
ros2 topic echo /robot_chatter
```

Output:
```text
data: Hello from robot A!
---
data: What's up, robot B?
---
data: Just sending some sensor readings!
```
:::

You can see messages as they are published. The `---` separates individual messages.

### Broadcasting Your Own Messages: `ros2 topic pub` 📢

Sometimes, you need to send a message yourself to test a node or control a robot. `ros2 topic pub` lets you publish data to any active topic. You'll specify the topic name, message type, and data.

:::info[Publish Message]
```bash
ros2 topic pub /robot_chatter std_msgs/String "data: 'Greetings, fellow robots!'"
```

Output:
```text
publisher: beginning loop
publishing #1: std_msgs.msg.String(data='Greetings, fellow robots!')
```
:::

This command publishes a `String` message. Remember to match the message type! You'll see confirmation that your message was published.

### Checking the Pulse of a Topic: `ros2 topic hz` ❤️

How often does a node publish messages? Is it a steady stream? `ros2 topic hz` tells you the publishing rate. Crucial for understanding performance.

:::info[Check Topic Rate]
```bash
ros2 topic hz /sensor_data
```

Output:
```text
average rate: 10.0
min: 0.09s max: 0.11s std dev: 0.01s window: 10
average rate: 9.9
min: 0.09s max: 0.12s std dev: 0.01s window: 10
```
:::

The output shows the average rate in Hz, with min, max, and standard deviation.

### Try With AI: Topic Explorers! 🧭

Now it's your turn to be a topic explorer!

> **💬 AI Colearning Prompt**: Imagine you have a new robot. What topics do you think it might have for navigation, and how would you use `ros2 topic list` and `ros2 topic echo` to understand them?

## Getting Started with ROS 2 Services

Hey there, future robot whisperer! 👋 Ready to give your robots direct commands?

That's where **ROS 2 Services** come in handy! They're like asking a question and getting an answer right back.

### How Services Work: Request & Reply 🤝

Think of Services as a client-server conversation. One node (the client) asks for something, and another node (the server) does it and replies! Perfect for one-time actions.

Here's the breakdown:

*   **Client**: Sends a request. Asks the server to perform a task.
*   **Server**: Receives request, does work, and sends reply.

### When to Use Services (Use Cases) 🤔

Services are awesome for specific, immediate tasks. Not for continuous data streams, but for those "do this now" moments!

*   **One-time commands**: Like telling a robot arm to "move to position X".
*   **Configuration changes**: Setting a sensor's sampling rate.
*   **Data queries**: Asking for a specific map coordinate.

### Robot Arm Example: A Service in Action 🤖

Let's control a robot arm. We'll create a service to move it to a specific angle.

#### Defining the Service Interface

First, define what our service will "talk" about using a `.srv` file.

```rosidl title="MoveArm.srv"
float32 angle
---
bool success
```

This means our request will have a `float32` called `angle`, and the reply will have a `bool` called `success`.

#### Creating a Service Server

Now, a Python node that acts as the server. It will wait for requests and simulate arm movement. We'll use `AddTwoInts` as a stand-in for custom service types in CLI examples.

```python title="arm_controller_server.py"
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Using a generic service for illustration

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.srv = self.create_service(AddTwoInts, 'move_arm', self.move_arm_callback)
        self.get_logger().info('Arm controller service ready!')

    def move_arm_callback(self, request, response):
        self.get_logger().info(f'Incoming request: {request.a} degrees')
        # Simulate arm movement
        if request.a >= 0 and request.a <= 180:
            response.sum = True # Assuming success
            self.get_logger().info(f'Arm moved to {request.a} degrees.')
        else:
            response.sum = False # Indicate failure
            self.get_logger().warn('Invalid angle requested.')
        return response

def main(args=None):
    rclpy.init(args=args)
    arm_controller = ArmController()
    rclpy.spin(arm_controller)
    arm_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Output:
```text
[INFO] [arm_controller]: Arm controller service ready!
[INFO] [arm_controller]: Incoming request: 90 degrees
[INFO] [arm_controller]: Arm moved to 90 degrees.
```

#### Making a Service Request

Finally, a client node that sends a request to our arm controller.

```python title="arm_client.py"
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Using a generic service for illustration
import sys

class ArmClient(Node):
    def __init__(self):
        super().__init__('arm_client')
        self.cli = self.create_client(AddTwoInts, 'move_arm')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.req = AddTwoInts.Request()

    def send_request(self, angle):
        self.req.a = angle
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    arm_client = ArmClient()
    angle_to_move = int(sys.argv[1]) if len(sys.argv) > 1 else 90
    arm_client.send_request(angle_to_move)

    while rclpy.ok():
        rclpy.spin_once(arm_client)
        if arm_client.future.done():
            try:
                response = arm_client.future.result()
            except Exception as e:
                arm_client.get_logger().error(f'Service call failed: {e}')
            else:
                if response.sum:
                    arm_client.get_logger().info(f'Arm movement successful!')
                else:
                    arm_client.get_logger().warn(f'Arm movement failed.')
            break

    arm_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Output:
```text
[INFO] [arm_client]: Service not available, waiting...
[INFO] [arm_client]: Arm movement successful!
```

### Exploring Services with `ros2 service` 🔍

ROS 2 provides command-line tools to interact with services. Useful for debugging and testing!

:::info[Listing Services]
See all active services in your ROS 2 system.

```bash
ros2 service list
```

Output:
```text
/arm_controller/move_arm
/parameter_events
/rosout
```
:::

This shows our `/arm_controller/move_arm` service is ready!

:::info[Calling a Service]
Manually call a service from the command line. Provide the service name, type, and request arguments in YAML.

```bash
ros2 service call /arm_controller/move_arm example_interfaces/srv/AddTwoInts "{a: 90}"
```

Output:
```text
requester: making request: example_interfaces.srv.AddTwoInts_Request(a=90)

response:
  sum: True
```
:::

This output shows the request sent and response received.

:::info[Finding Service Types]
If you know the service type but not the name, `ros2 service find` can help.

```bash
ros2 service find example_interfaces/srv/AddTwoInts
```

Output:
```text
/arm_controller/move_arm
```
:::

This confirms our service is of the `AddTwoInts` type.

### Try With AI 🚀

Now it's your turn! Can you modify a conceptual `MoveArm.srv` to also include a `speed` parameter for the robot arm? What changes would you need to make in the server and client nodes to accommodate this new parameter?

## Understanding ROS 2 Actions: Goal-Oriented Tasks 🎯

For complex, long-running tasks, like navigating to a room, ROS 2 Actions are your best friend! They are designed for goal-oriented communication.

Think of an Action as an enhanced Service. You send a goal, get continuous feedback, and receive a final result. Like ordering a pizza and getting updates: "dough is being made," "pizza in oven," "out for delivery," and finally, "delivered!"

### Action Components: Goal, Feedback, Result 🔄

An Action has three main parts:

*   **Goal**: The target you want the action server to achieve.
*   **Feedback**: Continuous updates on the progress of the goal.
*   **Result**: The final outcome of the goal.

### ROS 2 Action CLI Commands: Your Control Panel 🎮

Let's explore the command-line interface (CLI) tools to interact with ROS 2 Actions. These commands let you monitor, send goals, and track task progress.

:::info[Listing Available Actions]
See all actions your robot can perform. This command shows all active action servers.

```bash
ros2 action list
```

Output:
```text
/navigate_to_pose
/dock_robot
/follow_path
```
:::

This output lists actions like `/navigate_to_pose` for moving, `/dock_robot` for charging, and `/follow_path` for routes.

:::info[Sending a Goal]
Ready to give your robot a mission? This command sends a goal to an action server. You specify the action name, type, and goal details in YAML.

```bash
ros2 action send_goal /navigate_to_pose geometry_msgs/action/PoseStampedGoal "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

Output:
```text
Waiting for an action server to become available...
Sending goal:
    goal_id:
    stamp:
        sec: 1678881234
        nanosec: 567890123
    uuid: [0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}
    goal:
    pose:
        header:
        stamp:
            sec: 0
            nanosec: 0
        frame_id: map
        pose:
        position:
            x: 1.0
            y: 2.0
            z: 0.0
        orientation:
            x: 0.0
            y: 0.0
            z: 0.0
            w: 1.0
Feedback:
    current_distance_to_goal: 0.8
    status: Moving towards goal
Feedback:
    current_distance_to_goal: 0.5
    status: Adjusting orientation
Result:
    success: True
    message: Goal reached successfully
```
:::

This output shows the goal sent, continuous feedback, and the final result.

:::info[Monitoring Feedback (Conceptual)]
While `ros2 action send_goal` often displays feedback directly, sometimes you might monitor it separately. Understanding 'feedback' as a distinct channel is key.

```bash
# This command is conceptual for continuous feedback monitoring from another source.
# When using 'ros2 action send_goal', feedback is usually shown directly.
ros2 action feedback /navigate_to_pose
```

Conceptual Output:
```text
---
current_distance_to_goal: 0.7
status: Path planning...
---
current_distance_to_goal: 0.4
status: Executing path...
---
current_distance_to_goal: 0.1
status: Approaching target...
```
:::

This conceptual output illustrates how you receive progress updates.

### Try With AI: Your Action Plan! 🤖

Now it's time to brainstorm some robot actions!

> **💬 AI Colearning Prompt**: Imagine you are designing a cleaning robot. What kind of actions would it need, and how would you use `ros2 action list` and `ros2 action send_goal` to manage its tasks? Think about cleaning a room, emptying its dustbin, or returning to its charging station.

## Choosing the Right ROS 2 Communication

Understanding when to use Topics, Services, or Actions is key to building robust ROS 2 applications. Each mechanism shines in different situations. Let's explore some scenarios.

---

### Scenario 1: Sensor Data Stream

Imagine a robot continuously publishing LiDAR sensor data. This data needs to be available to multiple components: mapping, navigation, visualization. The data flow is one-way, with constant new readings.

> **💬 AI Colearning Prompt**: Which ROS 2 communication mechanism would you choose for this scenario, and why?

---

### Scenario 2: Robot Arm Movement

You need to command a robot arm to move to a specific joint configuration. The controller should acknowledge, execute, and report completion or failure. You also want to preempt movement if a higher-priority task arises.

> **💬 AI Colearning Prompt**: Which ROS 2 communication mechanism would you choose for this scenario, and why?

---

### Scenario 3: Map Generation Request

Your robot has a mapping system that can generate a new map on demand. When requested, the robot performs mapping and returns the completed map data. This is a single request-response, potentially taking time, but without real-time feedback during the process.

> **💬 AI Colearning Prompt**: Which ROS 2 communication mechanism would you choose for this scenario, and why?

---

### Scenario 4: Emergency Stop

A safety system needs an emergency stop button to immediately halt all robot operations. This is a critical, fire-and-forget command broadcast to all relevant components without waiting for acknowledgments.

> **💬 AI Colearning Prompt**: Which ROS 2 communication mechanism would you choose for this scenario, and why?

---

### Try With AI

Discuss your choices for each scenario with the AI. Explain your reasoning, and the AI will provide feedback and insights into the optimal communication mechanisms.

---
