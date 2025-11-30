---
sidebar_position: 5
sidebar_label: 'Lesson: 5 Describing Robots with URDF and Xacro'
slug: describing-robots-urdf-xacro
---

# Describing Robots with URDF and Xacro

To interact with and control a robot, whether it's a physical machine or a simulation, ROS 2 needs a precise digital description of that robot. This is where **URDF (Unified Robot Description Format)** comes in. URDF is an XML format used in ROS to describe all aspects of a robot: its physical structure, visual appearance, collision properties, and inertial characteristics.

Think of URDF as the robot's blueprint. It tells ROS 2 how different parts of the robot (links) are connected (joints), their sizes, masses, and where cameras or other sensors are located. This information is crucial for various robotics tasks, including:

*   **Visualization**: Displaying the robot accurately in 3D simulators like Gazebo or visualization tools like RViz2.
*   **Motion Planning**: Calculating how the robot can move from one point to another without self-collision or external collisions.
*   **Inverse Kinematics**: Determining the joint angles required to achieve a desired position and orientation of the robot's end-effector.
*   **Physics Simulation**: Simulating the robot's behavior under various physical forces.

## Understanding URDF Structure: Links and Joints

A URDF file is fundamentally a tree-like structure composed of two main elements: **links** and **joints**.

1.  **Links**: These represent the rigid bodies of your robot. A link has physical and visual properties.
    *   **Visual**: Describes how the link looks (e.g., its shape, color, texture). This is what you see in a simulator.
    *   **Collision**: Defines the link's physical boundaries for collision detection. This might be a simpler shape than the visual to speed up calculations.
    *   **Inertial**: Specifies the link's mass, center of mass, and moments of inertia, which are critical for physics simulations.

2.  **Joints**: These define how two links are connected and how they can move relative to each other. Joints introduce degrees of freedom to your robot.
    *   **Parent/Child**: Every joint connects a `parent` link to a `child` link.
    *   **Origin**: Defines the position and orientation of the child link's frame relative to the parent link's frame.
    *   **Axis**: For revolute (rotating) and prismatic (sliding) joints, this defines the axis of motion.
    *   **Type**: Specifies the kind of joint, such as:
        *   `revolute`: A rotating joint with a limited range of motion (e.g., a robot arm elbow).
        *   `continuous`: A rotating joint with unlimited range of motion (e.g., a wheel).
        *   `prismatic`: A sliding joint with a limited linear range (e.g., a linear actuator).
        *   `fixed`: No motion; the child link is rigidly attached to the parent (e.g., a robot's head rigidly attached to its body).

### Example: A Simple Two-Link Arm

Let's look at a basic URDF for a simple two-link robotic arm. This example defines a `base_link`, `link1`, and `link2`, connected by two `revolute` joints.

```xml title="simple_arm.urdf"
<?xml version="1.0"?>
<robot name="simple_arm">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint 1: Connects base_link to link1 -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="100"/>
  </joint>

  <!-- Link 1 -->
  <link name="link1">
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.2"/>
      </geometry>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.02" length="0.2"/>
      </geometry>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint 2: Connects link1 to link2 -->
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="100"/>
  </joint>

  <!-- Link 2 -->
  <link name="link2">
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.15"/>
      </geometry>
      <origin xyz="0 0 0.075" rpy="0 0 0"/>
      <material name="green">
        <color rgba="0 0.8 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.02" length="0.15"/>
      </geometry>
      <origin xyz="0 0 0.075" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.15"/>
      <origin xyz="0 0 0.075" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" izz="0.001"/>
    </inertial>
  </link>

</robot>
```

#### Conceptual Exercise: Analyzing a Simple URDF Model

Let's break down the `simple_arm.urdf` file to understand its components.

:::info[Analyze the simple_arm.urdf]
Carefully examine the `simple_arm.urdf` file above. Answer the following questions:

1.  How many links does the `simple_arm` robot have? Name them.
2.  How many joints does the `simple_arm` robot have? Name them and describe their types.
3.  What are the dimensions and material of the `base_link`?
4.  Where is `link1` attached relative to `base_link`? What is its shape and color?
5.  What is the range of motion for `joint2`?
6.  If you wanted to make `link2` shorter, which lines in the URDF would you modify?
:::

## Adding Visual and Collision Properties

The `visual` and `collision` tags within a `<link>` are fundamental for how your robot is perceived and interacts with its environment in simulation.

### Visual Properties

The `<visual>` tag defines the graphical representation of a link. This is what users will see when the robot is rendered in a visualization tool like RViz2 or a simulator like Gazebo.

Key elements within `<visual>`:

*   **`<geometry>`**: Specifies the shape of the visual element. Common geometries include:
    *   `<box size="X Y Z"/>`: A rectangular prism.
    *   `<cylinder radius="R" length="L"/>`: A cylinder.
    *   `<sphere radius="R"/>`: A sphere.
    *   `<mesh filename="package://your_package/meshes/your_mesh.stl" scale="X Y Z"/>`: For more complex shapes, you can import 3D models (e.g., STL, DAE).
*   **`<origin xyz="X Y Z" rpy="ROLL PITCH YAW"/>`**: Defines the pose (position and orientation) of the visual geometry relative to the link's own origin. `xyz` specifies translation, and `rpy` (roll, pitch, yaw) specifies rotation in radians.
*   **`<material>`**: Sets the color and/or texture of the visual element.
    *   `<color rgba="R G B A"/>`: Defines the color using RGBA values (Red, Green, Blue, Alpha, each from 0 to 1).

**Example:**
```xml
<link name="robot_base">
  <visual>
    <geometry>
      <box size="0.4 0.3 0.1"/>
    </geometry>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <material name="white">
      <color rgba="1 1 1 1"/>
    </material>
  </visual>
  <!-- ... other properties ... -->
</link>
```

### Collision Properties

The `<collision>` tag defines the physical shape of a link used for collision detection. It's often beneficial to use simpler shapes for collision than for visual representation to reduce computational load in simulations.

Key elements within `<collision>`:

*   **`<geometry>`**: Similar to visual geometry, but specifically for collision calculations. It uses the same types (`box`, `cylinder`, `sphere`, `mesh`).
*   **`<origin xyz="X Y Z" rpy="ROLL PITCH YAW"/>`**: Defines the pose of the collision geometry relative to the link's own origin.

**Example:**
```xml
<link name="robot_base">
  <!-- ... visual properties ... -->
  <collision>
    <geometry>
      <box size="0.4 0.3 0.1"/>
    </geometry>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </collision>
  <!-- ... other properties ... -->
</link>
```

**Why separate visual and collision?**
Imagine a robot with intricate designs, sharp edges, and complex curves for aesthetics (visual). If every detail were used for collision detection, the simulation would become incredibly slow. By using simpler, often convex, shapes for collision, you can maintain realism visually while ensuring efficient and stable physics simulations.

## Simplifying URDF with Xacro

Creating complex URDF files can sometimes feel like a lot of repetitive work. Imagine you have many identical parts, like robot fingers or wheel assemblies. Writing out all the `<link>` and `<joint>` details for each one can get tedious quickly. This is where **Xacro** comes to the rescue!

### What is Xacro?

Xacro, which stands for **XML Macros**, is an XML macro language. It allows you to use macros, properties, and mathematical expressions within your XML files. Specifically for URDF, Xacro helps you to:

*   **Reduce Redundancy**: Define common elements once and reuse them.
*   **Improve Readability**: Make your robot descriptions cleaner and easier to understand.
*   **Parameterize Models**: Use variables for dimensions or other properties, making your robot models more flexible.

Before ROS 2 uses your Xacro file, it first processes it to expand all the macros and substitute all properties. The final output is a standard URDF XML file.

### Xacro Properties: Keeping Things Consistent

Properties in Xacro are like variables in programming. You define them once, and then you can use them throughout your file. This is super helpful for maintaining consistent dimensions or colors.

To define a property, you use the `<xacro:property>` tag. To use it, you reference it with `$(property_name)`.

Here's an example:

```xml
<?xml version="1.0"?>
<robot name="my_xacro_robot" xmlns:xacro="http://ros.org/xacro">

  <xacro:property name="base_size" value="0.2" />
  <xacro:property name="wheel_radius" value="0.05" />

  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_size} ${base_size} ${base_size}" />
      </geometry>
    </visual>
  </link>

  <link name="left_wheel_link">
    <visual>
      <geometry>
        <cylinder radius="${wheel_radius}" length="0.02" />
      </geometry>
    </visual>
  </link>

</robot>
```

Output (simplified URDF after Xacro processing):

```xml
<?xml version="1.0"?>
<robot name="my_xacro_robot">

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.2" />
      </geometry>
    </visual>
  </link>

  <link name="left_wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.02" />
      </geometry>
    </visual>
  </link>

</robot>
```

Notice how `base_size` and `wheel_radius` are defined once and then used with `${}`. This makes it easy to change these values globally.

### Xacro Macros: Reusing Robot Components

Macros are even more powerful. They allow you to define entire XML blocks (like a `<link>` and its associated `<joint>`) as a reusable component. You can then "call" this macro multiple times, passing different arguments to customize each instance.

To define a macro, use `<xacro:macro name="macro_name" params="param1 param2">`. To use it, simply call `<xacro:macro_name param1="value1" param2="value2" />`.

Let's create a macro for a wheel assembly:

```xml
<?xml version="1.0"?>
<robot name="my_wheeled_robot" xmlns:xacro="http://ros.org/xacro">

  <xacro:property name="wheel_radius" value="0.05" />
  <xacro:property name="wheel_thickness" value="0.02" />

  <xacro:macro name="wheel" params="prefix parent_link x_offset y_offset z_offset">
    <link name="${prefix}_wheel_link">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_thickness}" />
        </geometry>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_thickness}" />
        </geometry>
      </collision>
      <inertial>
        <mass value="0.1" />
        <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" izz="0.0001" />
      </inertial>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent_link}" />
      <child link="${prefix}_wheel_link" />
      <origin xyz="${x_offset} ${y_offset} ${z_offset}" rpy="0 1.5708 1.5708" />
      <axis xyz="0 0 1" />
    </joint>
  </xacro:macro>

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.1" />
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.1" />
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0" />
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" izz="0.001" />
    </inertial>
  </link>

  <!-- Using the wheel macro to create two wheels -->
  <xacro:wheel prefix="right" parent_link="base_link" x_offset="0" y_offset="-0.12" z_offset="-0.05" />
  <xacro:wheel prefix="left" parent_link="base_link" x_offset="0" y_offset="0.12" z_offset="-0.05" />

</robot>
```

Output (partial URDF after Xacro processing for one wheel to show expansion):

```xml
<?xml version="1.0"?>
<robot name="my_wheeled_robot">

  <!-- ... base_link definition ... -->

  <link name="right_wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.02" />
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.02" />
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1" />
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" izz="0.0001" />
    </inertial>
  </link>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link" />
    <child link="right_wheel_link" />
    <origin xyz="0 -0.12 -0.05" rpy="0 1.5708 1.5708" />
    <axis xyz="0 0 1" />
  </joint>

  <!-- ... left_wheel_link and joint definitions would follow ... -->

</robot>
```

Xacro makes your robot descriptions modular, easier to manage, and less prone to errors when dealing with repetitive or parameterized structures.

## Visualizing URDF Models

After you define your robot using URDF or Xacro, you'll want to see it in action! Visualizing your robot model is essential for verifying its structure, checking joint limits, and understanding how different components relate to each other. It's also a crucial step for debugging any errors in your URDF.

### Introducing RViz2

The primary tool for visualizing URDF models in ROS 2 is **RViz2**. RViz2 is a powerful 3D visualization tool that allows you to display a robot model described by a URDF file. It's not just for static models; RViz2 can also show real-time sensor data, robot poses, path planning, and much more.

Here's how RViz2 helps you with URDF models:

*   **Displaying Robot Models**: RViz2 takes your URDF file and renders a 3D representation of your robot. You can see all the links and how they are connected by joints. This helps you confirm that your robot's physical structure is correctly defined.
*   **Joint State Visualization**: As your robot moves (either in simulation or real-world), RViz2 can visualize the current state of its joints. This means you can see the arm bending, wheels rotating, or any other joint motion as it happens.
*   **Coordinate Frames**: It can display the coordinate frames (axes) for each link and joint. This is incredibly useful for debugging. If your robot parts appear disconnected or misaligned, checking the coordinate frames can quickly reveal incorrect `origin` values in your URDF.
*   **Sensor Data Overlay**: Beyond just the robot model, RViz2 can overlay sensor data, such as camera feeds, lidar scans, or point clouds, directly onto the 3D scene. This helps in understanding sensor placement and coverage.
*   **Path and Trajectory Visualization**: When planning robot movements, RViz2 can visualize planned paths or trajectories, allowing you to see if the robot will navigate its environment as expected without collisions.

#### Conceptual Use for Debugging

Imagine you've created a complex URDF file, but when you load it, parts of your robot are floating in space or are attached incorrectly. RViz2 is your go-to tool for figuring out what went wrong.

*   **Check Link Hierarchy**: By visualizing the model, you can visually confirm that `parent` and `child` links are correctly connected by `joints`. If a link is missing or in the wrong place, it immediately stands out.
*   **Verify Origins**: The `origin` tags in URDF define the position and orientation of a child link relative to its parent, or the geometry relative to its link. If these values are incorrect, your robot will look distorted. RViz2 lets you see these displacements and rotations visually.
*   **Inspect Joint Types**: You can quickly see if a joint behaves as `revolute`, `prismatic`, or `fixed` by trying to move the robot (if connected to a simulator or real hardware). This helps catch errors where you might have intended a revolute joint but defined it as fixed.

In summary, RViz2 transforms your abstract URDF XML description into a tangible 3D model, providing immediate visual feedback that is invaluable for both design verification and troubleshooting.
