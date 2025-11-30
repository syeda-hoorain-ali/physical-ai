---
sidebar_position: 5
sidebar_label: 'Lesson 5: Bringing Robots to Life with ROS 2 Launch and Control'
slug: ros2-launch-and-control
---

# Lesson 5: Bringing Robots to Life with ROS 2 Launch and Control

In the exciting world of robotics, we need a way to orchestrate all the different components that make our robots move and think. This is where ROS 2 launch files and `ros2_control` come into play. They are like the conductor of an orchestra, ensuring every instrument plays its part perfectly.

## The Magic of ROS 2 Launch Files

Imagine you have a complex robot with many sensors, actuators, and software nodes. Manually starting each component would be a nightmare! ROS 2 launch files solve this by allowing you to define and execute multiple ROS 2 nodes and other commands with a single command. Think of them as blueprints for your robot's software ecosystem.

Launch files are especially crucial when you want to spawn your robot models into simulation environments like Gazebo. They tell Gazebo which URDF file to load, where to place the robot, and how to configure its controllers.

## Spawning Your URDF Model in Gazebo

Let's create a simple ROS 2 launch file to bring our URDF model to life in Gazebo. This launch file will start Gazebo and then load our robot description.

First, create a new file named `spawn_robot.launch.py` in your robot's ROS 2 package (e.g., `my_robot_description/launch`).

```python title="launch/spawn_robot.launch.py"
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the share directory for your robot package
    pkg_share_dir = get_package_share_directory('my_robot_description')

    # Path to your URDF file
    urdf_path = os.path.join(pkg_share_dir, 'urdf', 'my_robot.urdf')

    # Start Gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'gui': 'true'}.items(),
    )

    # Spawn the robot into Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', urdf_path, '-entity', 'my_robot'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_entity
    ])
```

> **💬 AI Colearning Prompt**: Run this launch file! What happens when you execute `ros2 launch my_robot_description spawn_robot.launch.py`?

Next, execute the launch file from your terminal:

```bash
ros2 launch my_robot_description spawn_robot.launch.py
```

Output:

```
[INFO] [launch]: All requested procedures have been executed
[gazebo_ros2_control-1] [INFO] [1678886400.123456789] [gazebo_ros2_control]: Loading ros2_control plugin
[spawn_entity.py-2] [INFO] [1678886400.987654321] [spawn_entity]: Spawning entity with name 'my_robot'
[spawn_entity.py-2] [INFO] [1678886401.000000000] [spawn_entity]: Entity 'my_robot' spawned successfully.
... (Gazebo GUI should open with your robot)
```

You should see Gazebo open with your robot model present in the simulation environment. This single command handles launching Gazebo and then injecting your robot.

## Configuring `ros2_control` for Your Robot

`ros2_control` is a powerful framework in ROS 2 that provides a standardized way to interface with robot hardware and simulate robot control. For our simulated robot, it allows us to send commands to its joints and read their states.

To use `ros2_control`, you need to:

1.  **Install the necessary packages**:
    ```bash
    sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control
    ```
    Output:

    ```
    Reading package lists... Done
    Building dependency tree... Done
    Reading state information... Done
    ros-humble-ros2-control is already the newest version (2.1.0-1jammy.20230303.234149).
    ros-humble-ros2-controllers is already the newest version (2.1.0-1jammy.20230303.235500).
    ros-humble-gazebo-ros2-control is already the newest version (0.6.0-1jammy.20230303.234150).
    0 upgraded, 0 newly installed, 0 to remove and 0 not upgraded.
    ```
2.  **Add `ros2_control` configuration to your URDF**: You need to define the controllers you want to use in your URDF file. This involves adding `ros2_control` tags that specify the robot's hardware interface.

    Here's an example of how to integrate `ros2_control` into your `my_robot.urdf` for a simple revolute joint:

    ```xml title="urdf\my_robot.urdf"
    <robot name="my_robot">
        <!-- ... existing links and joints ... -->

        <joint name="revolute_joint" type="revolute">
            <parent link="base_link"/>
            <child link="link_1"/>
            <origin xyz="0 0 0.1" rpy="0 0 0"/>
            <axis xyz="0 0 1"/>
            <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
        </joint>

        <!-- ros2_control configuration -->
        <ros2_control name="RobotHardware" type="system">
            <hardware>
                <plugin>gazebo_ros2_control/GazeboSystem</plugin>
            </hardware>
            <joint name="revolute_joint">
                <command_interface name="position">
                    <param name="min"> -1.57 </param>
                    <param name="max"> 1.57 </param>
                </command_interface>
                <state_interface name="position"/>
                <state_interface name="velocity"/>
            </joint>
        </ros2_control>

        <gazebo>
            <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
                <parameters>$(find my_robot_description)/config/my_robot_controllers.yaml</parameters>
            </plugin>
        </gazebo>

    </robot>
    ```

3.  **Create a controller configuration file**: This YAML file defines the specific controllers you want to load, such as joint position controllers. Create a file named `my_robot_controllers.yaml` in a `config` directory within your robot package.

```yaml title="config/my_robot_controllers.yaml"
    controller_manager:
      ros__parameters:
        update_rate: 100 # Hz

    joint_state_broadcaster:
      ros__parameters:
        type: joint_state_broadcaster/JointStateBroadcaster

    revolute_joint_position_controller:
      ros__parameters:
        type: position_controllers/JointPositionController
    ```

4.  **Update your launch file to load controllers**: Modify `spawn_robot.launch.py` to include the `ros2_control` controller manager and load your defined controllers.

    ```python title="launch\spawn_robot.launch.py"
    import os
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import LaunchConfiguration, Command
    from launch_ros.actions import Node

    def generate_launch_description():
        pkg_share_dir = get_package_share_directory('my_robot_description')
        urdf_path = os.path.join(pkg_share_dir, 'urdf', 'my_robot.urdf')
        controller_config_path = os.path.join(pkg_share_dir, 'config', 'my_robot_controllers.yaml')

        robot_description = Command(['xacro ', urdf_path])

        # Start Gazebo simulation
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
            launch_arguments={'gui': 'true'}.items(),
        )

        # Spawn the robot into Gazebo
        spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', urdf_path, '-entity', 'my_robot'],
            output='screen'
        )

        # Robot State Publisher
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        )

        # ros2_control controller manager
        control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, controller_config_path],
            output="screen",
        )

        # Load the joint state broadcaster
        joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        )

        # Load the joint position controller
        joint_position_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["revolute_joint_position_controller", "--controller-manager", "/controller_manager"],
        )

        return LaunchDescription([
            gazebo_launch,
            spawn_entity,
            robot_state_publisher_node,
            control_node,
            joint_state_broadcaster_spawner,
            joint_position_controller_spawner
        ])
```

> **💬 AI Colearning Prompt**: After making these changes, relaunch your robot. Do you see any new topics published related to joint states?

Execute the updated launch file from your terminal:

```bash
ros2 launch my_robot_description spawn_robot.launch.py
```

Output:

```
[INFO] [launch]: All requested procedures have been executed
[gazebo_ros2_control-1] [INFO] [1678886400.123456789] [gazebo_ros2_control]: Loading ros2_control plugin
[spawn_entity.py-2] [INFO] [1678886400.987654321] [spawn_entity]: Spawning entity with name 'my_robot'
[spawn_entity.py-2] [INFO] [1678886401.000000000] [spawn_entity]: Entity 'my_robot' spawned successfully.
[controller_manager-3] [INFO] [1678886401.500000000] [controller_manager]: Loading controller 'joint_state_broadcaster'
[spawner-4] [INFO] [1678886401.600000000] [joint_state_broadcaster]: Successfully loaded controller joint_state_broadcaster
[controller_manager-3] [INFO] [1678886401.700000000] [controller_manager]: Loading controller 'revolute_joint_position_controller'
[spawner-5] [INFO] [1678886401.800000000] [revolute_joint_position_controller]: Successfully loaded controller revolute_joint_position_controller
... (Gazebo GUI should open with your robot, and controllers are active)
```

Now, when you run the launch file, Gazebo will open, your robot will be spawned, and `ros2_control` will be active, ready to receive commands.

## Sending Joint Commands with a ROS 2 Publisher

With `ros2_control` configured, we can now send commands to our robot's joints. We'll create a simple ROS 2 publisher that sends position commands to our `revolute_joint`.

Create a new Python script named `joint_command_publisher.py` in your robot package's `scripts` directory (e.g., `my_robot_description/scripts`).

```python title="scripts\joint_command_publisher.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class JointCommandPublisher(Node):

    def __init__(self):
        super().__init__('joint_command_publisher')
        self.publisher_ = self.create_publisher(Float64, '/revolute_joint_position_controller/commands', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.position = 0.0
        self.direction = 1

    def timer_callback(self):
        msg = Float64()
        msg.data = self.position
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

        self.position += 0.1 * self.direction
        if self.position > 1.0 or self.position < -1.0:
            self.direction *= -1

def main(args=None):
    rclpy.init(args=args)
    joint_command_publisher = JointCommandPublisher()
    rclpy.spin(joint_command_publisher)
    joint_command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Make sure to make the script executable:

```bash
chmod +x my_robot_description/scripts/joint_command_publisher.py
```

On successful execution, this command typically produces no output.

Now, run this publisher node in a new terminal (after launching your robot):

```bash
ros2 run my_robot_description joint_command_publisher.py
```

Output:

```
[INFO] [joint_command_publisher]: Publishing: "0.0"
[INFO] [joint_command_publisher]: Publishing: "0.1"
[INFO] [joint_command_publisher]: Publishing: "0.2"
...
```

You should see the joint position commands being published to the `/revolute_joint_position_controller/commands` topic.

## Verifying Joint Commands

It's essential to verify that our commands are actually reaching the robot and influencing its behavior in Gazebo. We can do this using `ros2 topic echo` and by observing the robot's joint state in Gazebo.

1.  **Using `ros2 topic echo`**: In a new terminal, while your robot is launched and the publisher is running, use `ros2 topic echo` to listen to the joint state topic.

    ```bash
    ros2 topic echo /joint_states
    ```

    Output:

    ```
    header:
      stamp:
        sec: 1678886500
        nanosec: 123456789
      frame_id: ''
    name:
    - revolute_joint
    position:
    - 0.19999999999999998
    velocity:
    - 0.20000000000000001
    effort: []
    ---
    header:
      stamp:
        sec: 1678886500
        nanosec: 623456789
      frame_id: ''
    name:
    - revolute_joint
    position:
    - 0.29999999999999999
    velocity:
    - 0.19999999999999998
    effort: []
    ---
    ...
    ```

    You should see the `position` of the `revolute_joint` changing, reflecting the commands sent by our publisher.

2.  **Observing in Gazebo**: In the Gazebo simulation, you should physically see your robot's `revolute_joint` moving back and forth as the position commands are applied. This visual feedback confirms that the entire control pipeline is working correctly. You can also inspect the joint properties within Gazebo's interface to see its current position.

## Try With AI

> **💬 AI Colearning Prompt**: Experiment with the `joint_command_publisher.py` script.
> -   Change the `timer_period` to make the joint move faster or slower.
> -   Modify the `position` update logic to make the joint move to a specific target position and hold, instead of oscillating.
> -   How would you add another joint to your URDF and control it independently?
