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
