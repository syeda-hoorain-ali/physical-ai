---
sidebar_position: 1
sidebar_label: 'Lesson 1: Isaac Platform Setup and Configuration'
slug: isaac-platform-setup-and-configuration
---

# Lesson 1: Isaac Platform Setup and Configuration

## Learning Objectives

After completing this lesson, you will be able to:

- Verify hardware requirements for NVIDIA Isaac Sim 🖥️
- Install Isaac Sim on your development workstation 💻
- Configure Isaac ROS packages for robot development 🤖
- Troubleshoot common setup issues 🔧
- Validate your installation with basic tests ✅

These objectives align with CEFR B1-B2 levels and Bloom's taxonomy levels:
- Remember: Identify hardware requirements
- Understand: Explain installation process
- Apply: Execute setup procedures
- Analyze: Troubleshoot configuration issues

## Prerequisites

Before starting this lesson, ensure you have:

- Basic knowledge of ROS/ROS2 concepts 🤖
- Familiarity with Linux command line operations 💻
- Understanding of GPU computing concepts (CUDA) 📊
- Administrative access to your development machine 🔐

## Hardware Requirements Verification

### GPU Requirements

NVIDIA Isaac Sim requires a powerful GPU to run efficiently. Verify your system meets these requirements:

- **Minimum**: NVIDIA GPU with 8GB+ VRAM (RTX 2060 or equivalent)
- **Recommended**: NVIDIA GPU with 12GB+ VRAM (RTX 3080 or equivalent)
- **CUDA Compute Capability**: 6.0 or higher

:::info[Checking GPU Specifications]
```bash
nvidia-smi
```

**Output:**
```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 535.129.03   Driver Version: 535.129.03   CUDA Version: 12.2     |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|===============================+======================+======================|
|   0  NVIDIA RTX A4000    Off  | 00000000:01:00.0 Off |                  0 |
| 30%   34C    P8    14W / 140W |      1MiB / 16384MiB |      0%      Default |
+-------------------------------+----------------------+----------------------+
```
:::

### System Requirements Checklist

- [ ] NVIDIA GPU with CUDA Compute Capability 6.0+
- [ ] 16GB+ RAM (32GB recommended)
- [ ] 50GB+ free disk space
- [ ] Ubuntu 20.04 LTS or 22.04 LTS (or Windows 10/11 with WSL2)
- [ ] CPU with SSE 4.1 support
- [ ] Internet connection for downloads

![Hardware Requirements Diagram](/img/03-the-ai-robot-brain-nvidia-isaac/hardware-requirements.png "NVIDIA Isaac Hardware Requirements")

*Visual representation of the hardware requirements for Isaac Sim.*

The diagram above illustrates the essential hardware specifications needed to run Isaac Sim effectively. Understanding these requirements is crucial before proceeding with the installation process.

## Installing Isaac Sim

### Step 1: Install NVIDIA Drivers

:::info[Installing NVIDIA Drivers]
```bash
sudo apt update
sudo apt install nvidia-driver-535
sudo reboot
```

**Output:**
```
Hit:1 http://archive.ubuntu.com/ubuntu jammy InRelease
Hit:2 http://archive.ubuntu.com/ubuntu jammy-updates InRelease
...
The following NEW packages will be installed:
  nvidia-driver-535
...
Processing triggers for initramfs-tools (0.140ubuntu16) ...
update-initramfs: Generating /boot/initrd.img-5.15.0-76-generic
...
```
:::

### Step 2: Install CUDA Toolkit

:::info[Installing CUDA Toolkit for Isaac Sim]
```bash
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.0-1_all.deb
sudo dpkg -i cuda-keyring_1.0-1_all.deb
sudo apt-get update
sudo apt-get -y install cuda-toolkit-12-2
```

**Output:**
```
--2026-01-05 10:30:15--  https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.0-1_all.deb
Resolving developer.download.nvidia.com (developer.download.nvidia.com)... 152.195.153.133
...
Selecting previously unselected package cuda-keyring.
(Reading database ... 200000 files and directories currently installed.)
Preparing to unpack cuda-keyring_1.0-1_all.deb ...
Unpacking cuda-keyring (1.0-1) ...
...
The following additional packages will be installed: [CUDA packages]
...
```
:::

### Step 3: Download Isaac Sim

Navigate to the NVIDIA Isaac website to download Isaac Sim. You'll need to register for an NVIDIA Developer account if you don't have one.

:::info[Extracting Isaac Sim Package]
```bash
tar -xzf isaac_sim-2023.1.1.tar.gz
cd isaac_sim-2023.1.1
```

**Output:**
```
isaac_sim-2023.1.1/
isaac_sim-2023.1.1/isaac-sim.sh
isaac_sim-2023.1.1/install_dependencies.sh
isaac_sim-2023.1.1/isaac-sim-headless.setup
...
```
:::

### Step 4: Install Isaac Sim

Run the installation script:

```bash
./install_dependencies.sh
./isaac-sim-headless.setup
```

This process may take 20-30 minutes depending on your system.

**Output:**
```
Installing Isaac Sim dependencies...
Checking for required packages...
All dependencies satisfied.
Starting Isaac Sim setup...
Extracting assets... [####################] 100%
Setup complete. Isaac Sim is ready to use.
```

![Installation Process](/img/03-the-ai-robot-brain-nvidia-isaac/installation-process.png "Isaac Sim Installation Steps")

*Visual representation of the Isaac Sim installation process.*

### Step 5: Verify Installation

Launch Isaac Sim to verify the installation:

```bash
./isaac-sim.sh
```

You should see the Isaac Sim application start without errors.

**Output:**
```
Launching Isaac Sim...
Initializing CUDA context...
Loading Omniverse Kit...
Isaac Sim started successfully.
```

![Isaac Sim Interface](/img/03-the-ai-robot-brain-nvidia-isaac/isaac-sim-interface.png "Isaac Sim User Interface")

*Visual representation of the Isaac Sim interface after successful installation.*

## Isaac ROS Packages Installation

### Setting up ROS 2 Environment

First, install ROS 2 Humble Hawksbill:

![ROS Integration](/img/03-the-ai-robot-brain-nvidia-isaac/ros-integration.png "Isaac ROS Package Integration")

*The above diagram illustrates how Isaac packages integrate with the ROS ecosystem.*

:::info[Installing ROS 2 Humble Hawksbill]
```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions
```

**Output:**
```
Generating locales (this might take a while)...
  en_US.UTF-8... done
Generation complete.
```

**Output:**
```
[sudo] password for user:
```

**Output:**
```
Get:1 http://packages.ros.org/ros2/ubuntu jammy InRelease [4,637 B]
...
Fetched 234 MB in 2min 15s (1.73 MB/s)
Reading package lists... Done
Building dependency tree... Done
Reading state information... Done
The following additional packages will be installed: [list of packages]
...
```
:::

### Installing Isaac ROS Bridge

Clone and build the Isaac ROS packages:

:::info[Cloning Isaac ROS Packages]
```bash
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws/src

git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_bringup.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_perception.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
```

**Output:**
```
Cloning into 'isaac_ros_common'...
remote: Enumerating objects: 1245, done.
remote: Counting objects: 100% (342/342), done.
remote: Compressing objects: 100% (214/214), done.
remote: Total 1245 (delta 187), reused 201 (delta 119), pack-reused 903
Receiving objects: 100% (1245/1245), 1.23 MiB | 4.56 MiB/s, done.
Resolving deltas: 100% (678/678), done.
```
:::

:::info[Building Isaac ROS Workspace]
```bash
cd ~/isaac_ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

**Output:**
```
Starting >>> isaac_ros_common
Finished <<< isaac_ros_common [5.24s]
Starting >>> isaac_ros_bringup
Finished <<< isaac_ros_bringup [3.12s]
Starting >>> isaac_ros_image_pipeline
Finished <<< isaac_ros_image_pipeline [8.45s]
Summary: 3 packages finished [16.81s]
```
:::

## Troubleshooting Guide

### Common Issues and Solutions

#### Issue: CUDA Driver Version Mismatch

**Symptom**: Error about CUDA driver version mismatch
**Solution**:
1. Check your driver version: `nvidia-smi`
2. Install matching CUDA toolkit version
3. Reboot system after driver installation

#### Issue: OpenGL Context Creation Failure

**Symptom**: Isaac Sim fails to start with OpenGL errors
**Solution**:
1. Ensure your display manager is running
2. Try running with: `./isaac-sim.sh -- --headless`
3. Update your graphics drivers

#### Issue: Insufficient VRAM

**Symptom**: Application crashes with memory errors
**Solution**:
1. Close other GPU-intensive applications
2. Reduce Isaac Sim quality settings
3. Consider upgrading to a GPU with more VRAM

#### Issue: Isaac ROS Build Failures

**Symptom**: Colcon build fails with compilation errors
**Solution**:
1. Ensure all dependencies are installed: `rosdep install --from-paths src --ignore-src -r -y`
2. Clean build directory: `rm -rf build install log`
3. Rebuild: `colcon build --symlink-install`

## Verification Exercises

### Exercise 1: Hardware Check

Run the following command to verify your hardware setup:

```powershell
# Check GPU info
nvidia-smi

# Check CUDA installation
nvcc --version

# Check OpenGL support
glxinfo | grep -i opengl
```

**Output:**
```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 535.129.03   Driver Version: 535.129.03   CUDA Version: 12.2     |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|===============================+================------+================------+
|   0  NVIDIA RTX A4000    Off  | 00000000:01:00.0 Off |                  0 |
| 30%   34C    P8    14W / 140W |      1MiB / 16384MiB |      0%      Default |
+-------------------------------+----------------------+----------------------+
nvcc: NVIDIA (R) Cuda compiler driver
Copyright (c) 2005-2023 NVIDIA Corporation
Built on Tue_Sep_26_20:10:17_PDT_2023
Cuda compilation tools, release 12.3, V12.3.107
Build cuda_12.3.r12.3/compiler.33287557_0
```

### Exercise 2: Isaac Sim Test

Launch Isaac Sim and create a simple scene:

1. Open Isaac Sim
2. Create a new stage (File → New Stage)
3. Add a cube primitive (Create → Primitive → Cube)
4. Verify you can manipulate the cube with the transform tools

### Exercise 3: ROS Integration Test

Test the Isaac ROS bridge connection:

```bash
cd ~/isaac_ros_ws
source install/setup.bash
ros2 launch isaac_ros_bringup isaac_ros_common.launch.py
```

**Output:**
```
[INFO] [launch]: All log files can be found below /home/user/.ros/log/2026-01-05-10-35-42-123456
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [isaac_ros_common_node]: Isaac ROS Common node initialized
[INFO] [sensor_msgs]: Camera info loaded successfully
```

## Hands-On Activities

### Activity 1: System Optimization

Optimize your system for Isaac Sim performance:

1. Set GPU to maximum performance mode:
   ```bash
   sudo nvidia-smi -ac 5000,1590  # Adjust values for your GPU
   ```

2. Configure system for real-time performance:
   ```bash
   # Create the realtime group if it doesn't exist
   sudo groupadd -f realtime

   # Add your user to real-time group
   sudo usermod -a -G realtime $USER

   # Set real-time limits
   ulimit -r 99
   ```

   **Note:** If the 'realtime' group doesn't exist, you need to create it first using `groupadd -f realtime`. The `-f` flag ensures the command succeeds even if the group already exists.

### Activity 2: Isaac Sim Configuration

Configure Isaac Sim settings for your hardware:

1. Navigate to Isaac Sim directory
2. Edit the `isaac-sim.sh` file to add custom parameters
3. Add performance optimization flags:
   ```bash
   export ISAACSIM_FLAVOR=full
   export ISAACSIM_LICENSE_FILE=/path/to/license
   ```

### Activity 3: Custom Environment Setup

Create a custom environment in Isaac Sim:

1. Launch Isaac Sim
2. Create a new stage
3. Add lighting and environmental objects
4. Export the environment for later use

## Assessment Criteria

To successfully complete this lesson, you must demonstrate:

- [ ] Hardware requirements verification completed
- [ ] Isaac Sim installed and running without errors
- [ ] Isaac ROS packages built successfully
- [ ] Basic scene creation in Isaac Sim
- [ ] ROS communication test passed
- [ ] Troubleshooting techniques applied successfully

## Connection to Chapter Goals

This lesson establishes the foundation for all subsequent Isaac platform work. The skills learned here will enable you to:

- Develop and test robotic applications in simulation
- Integrate real sensors with simulated environments
- Prepare for advanced topics like visual SLAM and perception
- Bridge simulation to real-world robot deployment

## Explore

Now that you have Isaac Sim installed and configured, try creating your first robotic simulation environment. Experiment with different objects, lighting conditions, and physics properties to understand how they affect your simulation.

> **💬 AI Colearning Prompt**: Share your experience with the installation process. What challenges did you encounter, and how did you overcome them? What aspects of the Isaac platform are you most excited to explore?
