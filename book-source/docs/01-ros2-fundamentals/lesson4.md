---
sidebar_position: 4
sidebar_label: 'Lesson: 4 ROS 2 Services: Request and Response'
slug: ros2-services-request-and-response
---

# ROS 2 Services: Request and Response

In ROS 2, services provide a way for nodes to communicate by sending a request and receiving a response. This is a synchronous communication pattern, meaning the client waits for the server to respond before continuing. Think of it like calling a function on another computer.

## Understanding Service Communication

A ROS 2 service involves two main components:

*   **Service Server**: This node offers a service, waiting for requests from clients. When a request arrives, it performs an operation and sends back a response.
*   **Service Client**: This node sends a request to a service server and waits for the server's response.

This pattern is ideal for operations that require a single interaction and a direct result, such as asking a robot to perform a specific action and confirm its completion.

## Defining a Service Interface

Before creating a service, you need to define its interface. This specifies the data types for the request and response. ROS 2 uses `.srv` files for this.

For our examples, we will use the `AddTwoInts.srv` interface, which is defined as:

```srv title="AddTwoInts.srv"
int64 a
int64 b
---
int64 sum
```

This means the request will contain two `int64` values (`a` and `b`), and the response will contain one `int64` value (`sum`).

## Building a Service Server

Let's create a simple service server that adds two integers.

```python title="service_server.py"
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Service server started.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a = {request.a}, b = {request.b}. Sending response: {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

To run this server, you would typically use:

:::info[]
```bash
ros2 run <your_package_name> service_server
```

Output:
```
[INFO] [minimal_service]: Service server started.
[INFO] [minimal_service]: Incoming request: a = 2, b = 3. Sending response: 5
```
:::

## Creating a Service Client

Now, let's create a client that sends requests to our `add_two_ints` service.

```python title="service_client.py"
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    if len(sys.argv) == 3:
        a = int(sys.argv[1])
        b = int(sys.argv[2])
    else:
        minimal_client.get_logger().info('Usage: ros2 run <package_name> service_client <int_a> <int_b>')
        minimal_client.destroy_node()
        rclpy.shutdown()
        return

    response = minimal_client.send_request(a, b)
    minimal_client.get_logger().info(f'Result of add_two_ints: {response.sum}')

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

To run this client and send a request (e.g., adding 2 and 3):

:::info[]
```bash
ros2 run <your_package_name> service_client 2 3
```

Output:
```
[INFO] [minimal_client_async]: Result of add_two_ints: 5
```
:::

## Try With AI

> **💬 AI Colearning Prompt**: Experiment with the service client. What happens if you try to send a request before the service server is running? How would you modify the client to handle this gracefully?

## ROS 2 Actions: Goal-Oriented Tasks with Feedback

In ROS 2, actions extend the concept of services by providing long-running, goal-oriented tasks with periodic feedback and a final result. Imagine commanding a robot arm to pick up an object: you want to know if it's making progress (feedback) and if it successfully picked up the object (result).

### Defining an Action Interface

Similar to services, actions require a defined interface using `.action` files. For our examples, we'll use a `Fibonacci.action` interface:

```action title="Fibonacci.action"
int32 order
---
int32[] sequence
---
int32[] sequence
```

-   **Request (Goal)**: `int32 order` - The desired length of the Fibonacci sequence.
-   **Response (Result)**: `int32[] sequence` - The final computed Fibonacci sequence.
-   **Feedback**: `int32[] sequence` - The Fibonacci sequence computed so far.

### Building an Action Server

The action server processes goals, provides feedback, and eventually returns a result.

```python title="action_server.py"
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from rcl_interfaces.msg import SetParametersResult
from ros2_tutorials.action import Fibonacci # Replace with your action message type

import time

class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self.declare_parameter('goal_delay', 1.0) # Declare parameter for goal delay
        self.action_server = ActionServer(
            self,
            Fibonacci, # Replace with your action message type
            'fibonacci', # Action name
            self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback)
        self.get_logger().info('Fibonacci Action Server ready.')

    def destroy(self):
        self.action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info(f'Received goal request: {goal_request.order}')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        """Start execution of a goal."""
        self.get_logger().info('Goal accepted. Starting execution...')
        # Start a new thread or timer to execute the goal to avoid blocking
        # the main executor. For simplicity, we'll just call execute_callback directly here.
        self._execute_goal(goal_handle)

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info(f'Received cancel request for goal ID: {goal_handle.goal_id}')
        return CancelResponse.ACCEPT

    async def _execute_goal(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = Fibonacci.Feedback() # Replace with your action message type
        feedback_msg.sequence = []

        goal = goal_handle.request
        self.get_logger().info(f'Goal order: {goal.order}')

        # Start Fibonacci sequence
        a, b = 0, 1
        for i in range(goal.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled.')
                return

            feedback_msg.sequence.append(a)
            self.get_logger().info(f'Publishing feedback: {feedback_msg.sequence}')
            goal_handle.publish_feedback(feedback_msg)

            a, b = b, a + b
            time.sleep(self.get_parameter('goal_delay').get_parameter_value().double_value) # Use parameter for delay

        goal_handle.succeed()
        result = Fibonacci.Result() # Replace with your action message type
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Goal succeeded. Result: {result.sequence}')
        return result

    def execute_callback(self, goal_handle):
        """Execute a goal."""
        return rclpy.task.create_task(self._execute_goal(goal_handle))


def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)
    fibonacci_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

To run this server:

:::info[]
```bash
ros2 run <your_package_name> action_server
```

Output:

```
:::
[INFO] [fibonacci_action_server]: Fibonacci Action Server ready.
[INFO] [fibonacci_action_server]: Received goal request: 10
[INFO] [fibonacci_action_server]: Goal accepted. Starting execution...
[INFO] [fibonacci_action_server]: Executing goal...
[INFO] [fibonacci_action_server]: Goal order: 10
[INFO] [fibonacci_action_server]: Publishing feedback: [0]
[INFO] [fibonacci_action_server]: Publishing feedback: [0, 1]
[INFO] [fibonacci_action_server]: Publishing feedback: [0, 1, 1]
[INFO] [fibonacci_action_server]: Publishing feedback: [0, 1, 1, 2]
[INFO] [fibonacci_action_server]: Publishing feedback: [0, 1, 1, 2, 3]
[INFO] [fibonacci_action_server]: Publishing feedback: [0, 1, 1, 2, 3, 5]
[INFO] [fibonacci_action_server]: Publishing feedback: [0, 1, 1, 2, 3, 5, 8]
[INFO] [fibonacci_action_server]: Publishing feedback: [0, 1, 1, 2, 3, 5, 8, 13]
[INFO] [fibonacci_action_server]: Publishing feedback: [0, 1, 1, 2, 3, 5, 8, 13, 21]
[INFO] [fibonacci_action_server]: Publishing feedback: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34]
[INFO] [fibonacci_action_server]: Goal succeeded. Result: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34]
```

### Creating an Action Client

The action client sends a goal to the server, receives feedback, and waits for the final result.

```python title="action_client.py"
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from ros2_tutorials.action import Fibonacci # Replace with your action message type

class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci, # Replace with your action message type
            'fibonacci') # Action name
        self.get_logger().info('Fibonacci Action Client ready.')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal() # Replace with your action message type
        goal_msg.order = order

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info(f'Sending goal: {goal_msg.order}')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: {feedback_msg.feedback.sequence}')

def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()
    action_client.send_goal(10) # Example goal order
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```

To run this client (ensure the server is running first):

:::info[]
```bash
ros2 run <your_package_name> action_client
```

Output:

```
:::
[INFO] [fibonacci_action_client]: Fibonacci Action Client ready.
[INFO] [fibonacci_action_client]: Waiting for action server...
[INFO] [fibonacci_action_client]: Sending goal: 10
[INFO] [fibonacci_action_client]: Goal accepted :)
[INFO] [fibonacci_action_client]: Received feedback: [0]
[INFO] [fibonacci_action_client]: Received feedback: [0, 1]
[INFO] [fibonacci_action_client]: Received feedback: [0, 1, 1]
[INFO] [fibonacci_action_client]: Received feedback: [0, 1, 1, 2]
[INFO] [fibonacci_action_client]: Received feedback: [0, 1, 1, 2, 3]
[INFO] [fibonacci_action_client]: Received feedback: [0, 1, 1, 2, 3, 5]
[INFO] [fibonacci_action_client]: Received feedback: [0, 1, 1, 2, 3, 5, 8]
[INFO] [fibonacci_action_client]: Received feedback: [0, 1, 1, 2, 3, 5, 8, 13]
[INFO] [fibonacci_action_client]: Received feedback: [0, 1, 1, 2, 3, 5, 8, 13, 21]
[INFO] [fibonacci_action_client]: Received feedback: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34]
[INFO] [fibonacci_action_client]: Result: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34]
```

### Understanding Action Interactions

An action interaction is a powerful communication pattern. The client sends a goal, and the server works on it, sending back updates (feedback) as it progresses. Once the server finishes, it sends a final result. This allows for long-running, asynchronous operations where progress monitoring is crucial.

Consider a robot vacuum cleaner:
-   **Goal**: Clean the living room.
-   **Feedback**: "Cleaning zone A," "Battery at 75%," "Obstacle detected."
-   **Result**: "Living room cleaning complete," or "Cleaning aborted due to low battery."

### Choosing the Right Communication Pattern

ROS 2 offers Topics, Services, and Actions for inter-node communication. Selecting the right one depends on your specific needs.

*   **Topics**: Best for continuous, one-way streams of data.
    *   **Use Cases**: Sensor data (camera, lidar), robot odometry, joint states.
    *   **Analogy**: A radio broadcast where many listeners receive information, but don't respond directly.

*   **Services**: Ideal for synchronous, request-response interactions.
    *   **Use Cases**: Triggering a single robot action (e.g., "open gripper"), querying a parameter, simple calculations.
    *   **Analogy**: A function call where you send inputs and get a single return value.

*   **Actions**: Suited for long-running, goal-oriented tasks that require feedback and cancellability.
    *   **Use Cases**: Navigating to a target, complex manipulation sequences, running a long calculation with progress updates.
    *   **Analogy**: Ordering a custom meal at a restaurant. You place the order (goal), get updates on its preparation (feedback), and eventually receive your meal (result).
