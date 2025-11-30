---
sidebar_position: 4
sidebar_label: 'Lesson 4: Diving into Gazebo'
slug: diving-into-gazebo
---

# Lesson 4: Diving into Gazebo

Welcome to Gazebo, where your robots come to life in a virtual playground! This powerful simulator is a must-have for anyone working with robotics. Let's explore what makes it so special.

## What is Gazebo?

Gazebo is an open-source 3D robot simulator. It provides a robust environment to accurately and efficiently test your robot designs and algorithms. Think of it as a virtual lab where you can experiment without needing physical hardware.

Its primary purpose in robotics simulation is to offer a realistic testing ground. You can simulate complex scenarios and validate your control systems before deploying them to real robots. This saves time and resources, making development much smoother.

Key functionalities that make Gazebo shine:

*   **Physics Engine**: Real-world physics, including gravity, friction, and collisions.
*   **Sensor Simulation**: Mimics cameras, LiDAR, IMUs, and more for accurate data.
*   **Realistic Rendering**: Visualizes robots and environments with impressive detail.
*   **Plugin Interface**: Extend functionality with custom code for advanced tasks.

## Anatomy of a Gazebo World File (.world / .sdf)

Gazebo uses SDF (Simulation Description Format) files to define worlds and models. An SDF file describes everything from objects and robots to lighting and physics settings.

Let's look at the fundamental elements you'll find in a basic SDF structure:

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="simple_world">
    <!-- World elements go here -->
  </world>
</sdf>
```

**Output (Conceptual):**
This XML structure defines the blueprint for your simulated environment, ready to be populated with models, lights, and physics.

**Common elements within `<world>`:**

*   **`<model>`**: Defines a robot or any object within the world.
*   **`<include>`**: Reuses existing models from Gazebo's model database.
*   **`<light>`**: Configures light sources, like the sun.
*   **`<gui>`**: Customizes the Gazebo user interface.
*   **`<physics>`**: Sets up the simulation physics parameters.

## Building Your First World: A Step-by-Step Guide

Let's create a simple Gazebo world file. This will give you a hands-on feel for how these elements come together.

1.  **Create a new `.world` file**: Name it `simple_world.world` in your project directory (e.g., `my_robot_pkg/worlds/simple_world.world`).
2.  **Define a ground plane**: Every world needs a floor! This provides a surface for objects.
3.  **Add basic lighting**: Let there be light! A sun light makes your world visible and realistic.
4.  **Include simple walls or boundaries**: Create some simple boundaries to contain your robots.

Here's the complete `simple_world.world` content:

```xml title="simple_world.world"
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="simple_world">
    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
      </link>
    </model>

    <!-- Sun Light -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>

    <!-- Simple Walls -->
    <model name="wall_x_plus">
      <pose>5 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>0.1 10 1</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>0.1 10 1</size></box></geometry></visual>
      </link>
    </model>
    <model name="wall_x_minus">
      <pose>-5 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>0.1 10 1</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>0.1 10 1</size></box></geometry></visual>
      </link>
    </model>
    <model name="wall_y_plus">
      <pose>0 5 0.5 0 0 1.5707</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>0.1 10 1</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>0.1 10 1</size></box></geometry></visual>
      </link>
    </model>
    <model name="wall_y_minus">
      <pose>0 -5 0.5 0 0 1.5707</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>0.1 10 1</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>0.1 10 1</size></box></geometry></visual>
      </link>
    </model>
  </world>
</sdf>
```

**Output (Expected Visual in Gazebo):**
When you launch this world, you will see a grey ground plane, natural lighting from the sun, and four grey walls forming a square enclosure. The environment is now set for placing objects.

## Adding Basic Shapes: Cube and Cylinder

Now, let's add some primitive shapes to our world. We'll use a cube and a cylinder. The `<pose>` element is crucial for positioning and orienting your objects.

Add these `<model>` blocks inside your `<world>` tag in `simple_world.world`:

**Cube Example:**

```xml 
    <model name="my_cube">
      <pose>1 1 0.5 0 0 0</pose> <!-- x y z roll pitch yaw -->
      <link name="cube_link">
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Red</name>
            </script>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
```

**Output (Expected Visual in Gazebo):**
A red cube with sides of 1 meter will appear in your world at position (1, 1, 0.5) meters from the origin, with no rotation.

*   `<pose>1 1 0.5 0 0 0</pose>`: Sets the cube's position at (1, 1, 0.5) and no rotation.
*   `<box><size>1 1 1</size></box>`: Defines a cube with sides of 1 unit.

**Cylinder Example:**

```xml
    <model name="my_cylinder">
      <pose>-1 -1 0.5 0 0 0</pose>
      <link name="cylinder_link">
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Blue</name>
            </script>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
        </collision>
      </link>
    </model>
```

**Output (Expected Visual in Gazebo):**
A blue cylinder with a radius of 0.5 meters and a length of 1.0 meter will appear at (-1, -1, 0.5) meters, without rotation.

*   `<pose>-1 -1 0.5 0 0 0</pose>`: Positions the cylinder at (-1, -1, 0.5) with no rotation.
*   `<cylinder><radius>0.5</radius><length>1.0</length></cylinder>`: Creates a cylinder with a 0.5 radius and 1.0 length.

## Launching Your Gazebo World

After creating your `simple_world.world` file, you'll want to see it in action! Here's how to launch it from your command line.

Open your terminal and navigate to the directory containing `simple_world.world` (e.g., `D:\physical-ai`).

If you have ROS 2 installed and configured, you can use the following command:

```bash
ros2 launch gazebo_ros gazebo.launch.py gazebo_args:="-r -s libgazebo_ros_factory.so --world simple_world.world"
```

**Output (Expected Gazebo GUI Launch):**

```text
[INFO] [launch]: All log files can be found below /tmp/gazebo_logs/...
[INFO] [launch]: Default logging verbosity is set to INFO
... (other launch messages)
[INFO] [gazebo_ros-1] [INFO] [gz_ros_node.cpp:89] Waiting for connection to Gazebo...
[INFO] [gazebo_ros-1] [INFO] [gz_ros_node.cpp:89] Connected to Gazebo!
# (A new Gazebo GUI window should appear, showing your world with the ground plane, sun, walls, cube, and cylinder.)
```

Alternatively, if you don't have ROS 2 setup or prefer a direct Gazebo launch:

```bash
gazebo simple_world.world
```

**Output (Expected Gazebo GUI Launch):**

```text
[Msg] Waiting for master. [Msg] Connected to gazebo master @ http://127.0.0.1:11345
[Msg] Publicized address: 192.168.1.100
# (A new Gazebo GUI window should appear, showing your world with the ground plane, sun, walls, cube, and cylinder.)
```

This will open the Gazebo GUI, displaying your newly created world with all the elements you defined.

## Try With AI

> **💬 AI Colearning Prompt**: Experiment with the `simple_world.world` file. Try changing the `pose` of one of the walls or resizing the cube. How does this impact the visual outcome in Gazebo?
