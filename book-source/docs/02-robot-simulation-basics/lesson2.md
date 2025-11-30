---
sidebar_position: 2
sidebar_label: 'Lesson 2: Anatomy of a Robot - Links and Joints'
slug: anatomy-of-a-robot-links-and-joints
---

# Lesson 2: Anatomy of a Robot - Links and Joints

Ever wondered how a robot moves? It all boils down to its fundamental building blocks: links and joints. Think of links as the robot's "bones" and joints as its "articulations" or "pivot points." Together, they create the complex movements we see in everything from industrial arms to humanoid robots.

## Understanding Links (Rigid Bodies)

Links are the rigid parts of a robot. They don't bend or change shape. Each link has its own physical properties like mass, inertia, and visual characteristics. In a robot simulation, these properties are crucial for accurate physics and rendering.

### The `<link>` Tag: Defining Robot Segments

The `<link>` tag in URDF (Unified Robot Description Format) defines a single rigid body of the robot. It has several important sub-elements:

*   **`<visual>`**: Describes how the link looks.
    *   `<geometry>`: Defines the shape (e.g., box, cylinder, sphere, mesh).
    *   `<material>`: Sets the color and texture.
*   **`<collision>`**: Defines the link's physical boundaries for collision detection.
    *   `<geometry>`: Similar to visual, but for collision calculations.
*   **`<inertial>`**: Specifies the physical properties that affect dynamics.
    *   `<mass>`: The mass of the link.
    *   `<inertia>`: How hard it is to change the link's rotation.

```xml
<!-- Define a rigid body named "base_link" -->
<link name="base_link">
  <!-- Visual properties of the link (how it looks) -->
  <visual>
    <!-- Geometry of the visual component: a box with specified dimensions -->
    <geometry>
      <box size="0.6 0.4 0.2"/>
    </geometry>
    <!-- Material properties: a blue color -->
    <material name="blue">
      <color rgba="0 0 0.8 1"/>
    </material>
  </visual>
  <!-- Collision properties of the link (how it interacts physically) -->
  <collision>
    <!-- Geometry for collision detection: a box matching the visual -->
    <geometry>
      <box size="0.6 0.4 0.2"/>
    </geometry>
  </collision>
  <!-- Inertial properties of the link (for physics calculations) -->
  <inertial>
    <!-- Mass of the link in kilograms -->
    <mass value="10"/>
    <!-- Inertia tensor (simplified for a box) -->
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
  </inertial>
</link>
```

Output:
The XML snippet above defines a link named "base_link" with a blue box shape for both visual and collision properties, and specifies its mass and inertia.

## Understanding Joints (Connections)

Joints are the mechanical connections between two links. They define how one link moves relative to another. Without joints, a robot would just be a collection of static parts. Joints are what bring movement to a robot.

### The `<joint>` Tag: Connecting Robot Segments

The `<joint>` tag connects two links, specifying their relationship and degrees of freedom.

*   **`name`**: A unique identifier for the joint.
*   **`type`**: The kind of movement the joint allows. Common types include:
    *   `revolute`: Rotational movement around a single axis (like a hinge).
    *   `prismatic`: Linear movement along a single axis (like a piston).
    *   `fixed`: No movement; rigidly connects two links.
    *   `continuous`: Revolute joint with unlimited range.
*   **`<parent>`**: Specifies the name of the parent link.
*   **`<child>`**: Specifies the name of the child link.
*   **`<origin>`**: Defines the joint's position and orientation relative to the parent link.
    *   `xyz`: The x, y, z coordinates.
    *   `rpy`: Roll, pitch, yaw (rotation) in radians.
*   **`<axis>`**: Defines the axis of rotation or translation for revolute/prismatic joints.
    *   `xyz`: The x, y, z components of the axis vector.
*   **`<limit>`**: For revolute and prismatic joints, sets the range of motion.
    *   `lower`, `upper`: Minimum and maximum joint positions.
    *   `effort`, `velocity`: Maximum force/torque and velocity.

```xml
<!-- Define a revolute joint named "joint1" -->
<joint name="joint1" type="revolute">
  <!-- Parent link to which this joint is attached -->
  <parent link="base_link"/>
  <!-- Child link that moves relative to the parent -->
  <child link="link1"/>
  <!-- Position and orientation of the joint relative to the parent link's origin -->
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <!-- Axis of rotation for the revolute joint (Z-axis in this case) -->
  <axis xyz="0 0 1"/>
  <!-- Limits the range of motion for the joint -->
  <limit lower="-1.57" upper="1.57" effort="100" velocity="100"/>
</joint>
```

Output:
The XML above defines a revolute joint named "joint1" connecting "base_link" to "link1", positioned at (0,0,0.1) relative to "base_link", rotating around the Z-axis, with a limited range of motion.

## Building Your First 2-Link Robot URDF

Let's put theory into practice by creating a simple 2-link robot. This robot will have a `base_link`, a `link1`, and a `joint1` connecting them.

### Step-by-Step Tutorial

1.  **Create a new file**: In your project's URDF directory (e.g., `my_robot_description/urdf`), create a file named `two_link_robot.urdf`. Copy the full code example provided below into this file.
2.  **Define the robot tag**: Every URDF file starts with a `<robot>` tag and a name.

    ```xml
    <?xml version="1.0"?>
    <!-- The root tag for all URDF robot descriptions -->
    <robot name="simple_arm">
        <!-- Links and Joints will go here -->
    </robot>
    ```
    Output:
    This XML sets up the basic structure for any URDF file, declaring a robot named "simple_arm".

3.  **Add the `base_link`**: This is our first rigid body. We'll make it a simple box.

    ```xml
    <!-- Define the base link -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.6 0.4 0.2"/>
            </geometry>
            <material name="blue">
                <color rgba="0 0 0.8 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <box size="0.6 0.4 0.2"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="10"/>
            <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
        </inertial>
    </link>
    ```
    Output:
    This defines the robot's stationary base with specific visual, collision, and inertial properties.

4.  **Add `link1`**: Our second link, another box, connected to the base.

    ```xml
    <!-- Define the first link connected to the base -->
    <link name="link1">
        <visual>
            <geometry>
                <box size="0.1 0.1 0.5"/>
            </geometry>
            <material name="green">
                <color rgba="0 0.8 0 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <box size="0.1 0.1 0.5"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="1"/>
            <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
        </inertial>
    </link>
    ```
    Output:
    This creates a second link, `link1`, distinct in size and color, ready to be attached.

5.  **Connect `base_link` and `link1` with `joint1`**: This revolute joint will allow `link1` to rotate relative to `base_link`.

    ```xml
    <!-- Define the first joint connecting base_link and link1 -->
    <joint name="joint1" type="revolute">
        <parent link="base_link"/>
        <child link="link1"/>
        <origin xyz="0 0 0.1" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1.57" upper="1.57" effort="100" velocity="100"/>
    </joint>
    ```

### Full Code Example: `two_link_robot.urdf`

```xml
<?xml version="1.0"?>
<robot name="simple_arm">

    <!-- Base Link: The stationary base of the robot -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.6 0.4 0.2"/>
            </geometry>
            <material name="blue">
                <color rgba="0 0 0.8 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <box size="0.6 0.4 0.2"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="10"/>
            <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
        </inertial>
    </link>

    <!-- Link 1: The first moving segment of the arm -->
    <link name="link1">
        <visual>
            <geometry>
                <box size="0.1 0.1 0.5"/>
            </geometry>
            <material name="green">
                <color rgba="0 0.8 0 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <box size="0.1 0.1 0.5"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="1"/>
            <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
        </inertial>
    </link>

    <!-- Joint 1: Connects base_link to link1, allowing rotation around Z-axis -->
    <joint name="joint1" type="revolute">
        <parent link="base_link"/>
        <child link="link1"/>
        <!-- Origin defines the joint's position and orientation relative to the parent -->
        <origin xyz="0 0 0.1" rpy="0 0 0"/>
        <!-- Axis defines the rotation axis for revolute joints -->
        <axis xyz="0 0 1"/>
        <!-- Limits define the min/max angle, and max effort/velocity -->
        <limit lower="-1.57" upper="1.57" effort="100" velocity="100"/>
    </joint>

</robot>
