from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates a ROS 2 launch description to start a publisher and a subscriber node.

    This launch file starts two nodes from the `py_pubsub` package:
    - `talker` (publisher node)
    - `listener` (subscriber node)
    """
    return LaunchDescription([
        Node(
            package='py_pubsub',
            executable='publisher',
            name='talker',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='py_pubsub',
            executable='subscriber',
            name='listener',
            output='screen',
            emulate_tty=True
        )
    ])
