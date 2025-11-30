---
sidebar_position: 1
sidebar_label: 'Lesson 1: Prerequisites and Environment Setup'
slug: prerequisites-environment-setup
---

# Prerequisites for Robot Simulation

This section outlines the necessary software and tools required to follow along with the robot simulation lessons. It is highly recommended to use a Linux-based operating system (e.g., Ubuntu 22.04 LTS) for the best experience with ROS 2 and Gazebo.

## Option 1: Using a Pre-configured Docker Image (Recommended)

For ease of setup and to avoid dependency conflicts, it is recommended to use a pre-configured Docker image. This will provide a consistent environment with all necessary tools already installed.

**Coming Soon:** Details on how to pull and run the official Docker image will be provided here.

## Option 2: Manual Installation

If you prefer a manual setup, please follow the instructions below.

### 1. Ubuntu Installation

Ensure you have a fresh installation of Ubuntu 22.04 LTS (Jammy Jellyfish).

### 2. ROS 2 Humble Hawksbill Installation

Follow the official ROS 2 documentation for installing Humble Hawksbill on Ubuntu:

[ROS 2 Humble Hawksbill Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

Key steps include:

*   Setting up locales
*   Setting up sources
*   Adding the GPG key
*   Installing ROS 2 packages (`ros-humble-desktop`)
*   Sourcing the setup script

### 3. Gazebo Garden Installation

Gazebo Garden is the default simulator for ROS 2 Humble. Follow the official Gazebo documentation for installation:

[Gazebo Garden Installation Guide](https://gazebosim.org/docs/garden/install_ubuntu)

Key steps include:

*   Setting up sources
*   Adding the GPG key
*   Installing Gazebo packages (`gz-garden`)

### 4. ROS 2 Development Tools

Install common ROS 2 development tools:

```bash
sudo apt update
sudo apt install -y ros-humble-colcon-common-extensions ros-humble-rqt* ros-humble-rviz2 ros-humble-joint-state-publisher-gui
```

### 5. `ros2_control` Installation

Install the `ros2_control` packages:

```bash
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers
```

### 6. VS Code (Optional)

Install Visual Studio Code for development:

[VS Code Installation Guide](https://code.visualstudio.com/docs/setup/linux)

Recommended VS Code Extensions:

*   ROS 2
*   C/C++ Extension Pack
*   Python
*   Markdown All in One

### 7. Other Dependencies

Some tutorials might require additional Python packages or system dependencies. These will be mentioned as needed within the relevant lessons.
