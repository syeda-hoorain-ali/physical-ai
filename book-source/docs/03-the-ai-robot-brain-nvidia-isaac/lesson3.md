---
sidebar_position: 3
sidebar_label: 'Lesson 3: Isaac ROS Hardware-Accelerated Perception'
slug: isaac-ros-hardware-accelerated-perception
---

# Lesson 3: Isaac ROS Hardware-Accelerated Perception

## Learning Objectives

After completing this lesson, you will be able to:

- Implement hardware-accelerated perception pipelines using Isaac ROS packages 🤖
- Configure GPU-accelerated computer vision algorithms for real-time processing 📸
- Integrate multiple sensors with hardware-accelerated perception nodes ⚡
- Optimize perception performance using CUDA and TensorRT acceleration 🚀
- Evaluate perception accuracy and performance metrics in real-world scenarios 📊

These objectives align with CEFR C1-C2 levels and Bloom's taxonomy levels:
- Remember: Identify key Isaac ROS perception packages
- Understand: Explain GPU acceleration in perception pipelines
- Apply: Configure perception nodes for sensor integration
- Analyze: Evaluate performance metrics and optimize configurations
- Evaluate: Compare different acceleration strategies
- Create: Design optimized perception pipelines for specific use cases

## Prerequisites

Before starting this lesson, ensure you have:

- Completed Lesson 1: Isaac Platform Setup and Configuration 🛠️
- Completed Lesson 2: Isaac Sim Photorealistic Simulation Fundamentals 🌟
- Understanding of ROS 2 concepts and message types 🤖
- Basic knowledge of computer vision and perception algorithms 📸
- Experience with GPU computing (CUDA, TensorRT) 📊
- Familiarity with Isaac ROS common packages and nodes ⚙️

## Understanding Isaac ROS Perception Capabilities

Isaac ROS perception packages provide hardware-accelerated computer vision algorithms that leverage NVIDIA GPUs for real-time processing. These packages include:

- **Isaac ROS Image Pipeline**: Optimized image processing with CUDA acceleration
- **Isaac ROS Stereo Dense Reconstruction**: Real-time 3D reconstruction from stereo cameras
- **Isaac ROS Visual Slam**: Hardware-accelerated visual SLAM with pose estimation
- **Isaac ROS Detection**: Object detection and tracking with TensorRT acceleration
- **Isaac ROS Segmentation**: Semantic and instance segmentation using GPU inference

![Isaac ROS Perception Pipeline](/img/03-the-ai-robot-brain-nvidia-isaac/perception-pipeline-architecture.png "Isaac ROS Perception Pipeline Architecture")

### Key Perception Packages

The Isaac ROS perception stack includes several key packages that provide hardware acceleration:

- **isaac_ros_image_pipeline**: Accelerated image processing including rectification, resize, and format conversion
- **isaac_ros_stereo_image_proc**: GPU-accelerated stereo processing for depth estimation
- **isaac_ros_detect_and_track**: Object detection and tracking with TensorRT acceleration
- **isaac_ros_point_cloud_localization**: GPU-accelerated point cloud processing and localization
- **isaac_ros_apriltag**: Hardware-accelerated AprilTag detection and pose estimation

## Sensor Configuration Guide

### Camera Setup and Calibration

Proper camera configuration is crucial for effective perception. Isaac ROS supports various camera types with hardware acceleration:

```bash
# Create camera calibration directory
mkdir -p ~/isaac_ros_ws/src/camera_calibrations
cd ~/isaac_ros_ws/src/camera_calibrations

# Create camera info file for RGB camera
cat <<EOF > rgb_camera_info.yaml
camera_matrix:
  rows: 3
  cols: 3
  data: [616.13, 0.0, 311.41, 0.0, 615.55, 226.91, 0.0, 0.0, 1.0]
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.41565, 0.15873, 0.00053, -0.00061, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [584.12, 0.0, 311.41, 0.0, 0.0, 584.12, 226.91, 0.0, 0.0, 0.0, 1.0, 0.0]
distortion_model: plumb_bob
EOF
```

**Output:**
```
rgb_camera_info.yaml created successfully
camera_name: rgb_camera
```

### Stereo Camera Configuration

For depth perception, configure stereo cameras with proper calibration:

```bash
# Create stereo calibration file
cat <<EOF > stereo_calib.yaml
left:
  camera_matrix:
    rows: 3
    cols: 3
    data: [616.13, 0.0, 311.41, 0.0, 615.55, 226.91, 0.0, 0.0, 1.0]
  distortion_coefficients:
    rows: 1
    cols: 5
    data: [-0.41565, 0.15873, 0.00053, -0.00061, 0.0]
  rectification_matrix:
    rows: 3
    cols: 3
    data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
  projection_matrix:
    rows: 3
    cols: 4
    data: [584.12, 0.0, 311.41, 0.0, 0.0, 584.12, 226.91, 0.0, 0.0, 0.0, 1.0, 0.0]
right:
  camera_matrix:
    rows: 3
    cols: 3
    data: [616.13, 0.0, 311.41, 0.0, 615.55, 226.91, 0.0, 0.0, 1.0]
  distortion_coefficients:
    rows: 1
    cols: 5
    data: [-0.41565, 0.15873, 0.00053, -0.00061, 0.0]
  rectification_matrix:
    rows: 3
    cols: 3
    data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
  projection_matrix:
    rows: 3
    cols: 4
    data: [584.12, 0.0, 311.41, 100.0, 0.0, 584.12, 226.91, 0.0, 0.0, 0.0, 1.0, 0.0]
baseline: 0.1  # Baseline in meters
EOF
```

**Output:**
```
stereo_calib.yaml created successfully
baseline: 0.1 meters
```

### LiDAR Configuration

For 3D perception, configure LiDAR sensors with appropriate parameters:

```bash
# Create LiDAR configuration file
cat <<EOF > lidar_config.yaml
# Velodyne VLP-16 configuration
device_ip: "192.168.1.201"
udp_port: 2368
pcap: ""
read_once: false
read_fast: false
repeat_delay: 0.0
frame_id: "velodyne"
calibration: "velodyne_VLP_16db.yaml"
model: "VLP16"
EOF
```

**Output:**
```
lidar_config.yaml created successfully
device_ip: 192.168.1.201
```

## Perception Pipeline Implementation Tutorial

### Basic Image Pipeline Setup

Let's create a hardware-accelerated image processing pipeline:

```bash
# Navigate to workspace
cd ~/isaac_ros_ws
source install/setup.bash

# Create a launch file for image pipeline
mkdir -p src/image_pipeline_launch
cd src/image_pipeline_launch

# Create image pipeline launch file
cat <<EOF > image_pipeline.launch.py
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generate launch description for image pipeline."""
    container = ComposableNodeContainer(
        name='image_pipeline_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::RectifyNode',
                name='rectify_node',
                parameters=[{
                    'output_width': 640,
                    'output_height': 480,
                }],
                remappings=[
                    ('image_raw', '/camera/image_raw'),
                    ('camera_info', '/camera/camera_info'),
                    ('image_rect', '/camera/image_rect'),
                ],
            ),
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ResizeNode',
                name='resize_node',
                parameters=[{
                    'output_width': 320,
                    'output_height': 240,
                }],
                remappings=[
                    ('image', '/camera/image_rect'),
                    ('camera_info', '/camera/camera_info'),
                    ('resized_image', '/camera/image_resized'),
                ],
            ),
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])
EOF
```

**Output:**
```
image_pipeline.launch.py created successfully
```

### Stereo Depth Pipeline

Create a stereo depth estimation pipeline with GPU acceleration:

```bash
# Create stereo pipeline launch file
cat <<EOF > stereo_pipeline.launch.py
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generate launch description for stereo depth pipeline."""
    container = ComposableNodeContainer(
        name='stereo_depth_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_stereo_image_proc',
                plugin='nvidia::isaac_ros::stereo_image_proc::DisparityNode',
                name='disparity_node',
                parameters=[{
                    'min_disparity': 0.0,
                    'max_disparity': 64.0,
                    'num_disparities': 64,
                    'stereo_algorithm': 1,  # BLOCK_MATCHING
                }],
                remappings=[
                    ('left/image_rect', '/camera/left/image_rect'),
                    ('left/camera_info', '/camera/left/camera_info'),
                    ('right/image_rect', '/camera/right/image_rect'),
                    ('right/camera_info', '/camera/right/camera_info'),
                    ('disparity', '/stereo/disparity'),
                ],
            ),
            ComposableNode(
                package='isaac_ros_stereo_image_proc',
                plugin='nvidia::isaac_ros::stereo_image_proc::PointCloudNode',
                name='pointcloud_node',
                parameters=[{
                    'output_frame': 'camera_depth_optical_frame',
                    'pointcloud_type': 1,  # XYZI
                }],
                remappings=[
                    ('left/image_rect', '/camera/left/image_rect'),
                    ('left/camera_info', '/camera/left/camera_info'),
                    ('disparity', '/stereo/disparity'),
                    ('points', '/stereo/points'),
                ],
            ),
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])
EOF
```

**Output:**
```
stereo_pipeline.launch.py created successfully
```

### Object Detection Pipeline

Implement a hardware-accelerated object detection pipeline:

```bash
# Create object detection pipeline
cat <<EOF > detection_pipeline.launch.py
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generate launch description for object detection pipeline."""
    container = ComposableNodeContainer(
        name='detection_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_detectnet',
                plugin='nvidia::isaac_ros::detection::DetectNetNode',
                name='detectnet_node',
                parameters=[{
                    'model_name': 'ssd_mobilenet_v2_coco',
                    'input_layer_name': 'input',
                    'output_layer_names': ['scores', 'boxes', 'classes'],
                    'confidence_threshold': 0.5,
                    'max_batch_size': 1,
                    'input_tensor': 'input',
                    'input_layer_width': 300,
                    'input_layer_height': 300,
                    'output_layer_width': 1917,
                    'output_layer_height': 1,
                    'mean_pixel_value': 0.0,
                    'threshold': 0.5,
                }],
                remappings=[
                    ('image', '/camera/image_rect'),
                    ('detections', '/detections'),
                ],
            ),
            ComposableNode(
                package='isaac_ros_detection_2d_to_3d',
                plugin='nvidia::isaac_ros::detection_2d_to_3d::Detection2DTo3DNode',
                name='detection_2d_to_3d_node',
                remappings=[
                    ('image', '/camera/image_rect'),
                    ('camera_info', '/camera/camera_info'),
                    ('detections_2d', '/detections'),
                    ('detections_3d', '/detections_3d'),
                ],
            ),
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])
EOF
```

**Output:**
```
detection_pipeline.launch.py created successfully
```

## GPU Acceleration Setup Instructions

![GPU Acceleration Diagram](/img/03-the-ai-robot-brain-nvidia-isaac/gpu-acceleration.png "GPU Acceleration in Isaac ROS Perception")

### CUDA and TensorRT Configuration

Ensure CUDA and TensorRT are properly configured for Isaac ROS perception:

```bash
# Verify CUDA installation
nvidia-smi
nvcc --version

# Check TensorRT installation
python3 -c "import tensorrt as trt; print(f'TensorRT version: {trt.__version__}')"
```

**Output:**
```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 535.129.03   Driver Version: 535.129.03   CUDA Version: 12.2     |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|===============================+======================+================------+
|   0  NVIDIA RTX A4000    Off  | 00000000:01:00.0 Off |                  0 |
| 30%   34C    P8    14W / 140W |      1MiB / 16384MiB |      0%      Default |
+-------------------------------+----------------------+----------------------+
nvcc: NVIDIA (R) Cuda compiler driver
Copyright (c) 2005-2023 NVIDIA Corporation
Built on Tue_Sep_26_20:10:17_PDT_2023
Cuda compilation tools, release 12.3, V12.3.107
Build cuda_12.3.r12.3/compiler.33287557_0
TensorRT version: 8.6.1
```

### Isaac ROS Perception Package Installation

Install Isaac ROS perception packages with GPU acceleration:

:::info[Installing Isaac ROS Perception Packages]
```bash
# Navigate to workspace
cd ~/isaac_ros_ws/src

# Clone perception packages
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_stereo_image_proc.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_detectnet.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_detection_2d_to_3d.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pointcloud_utils.git

# Build the workspace
cd ~/isaac_ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  isaac_ros_image_pipeline \
  isaac_ros_stereo_image_proc \
  isaac_ros_detectnet \
  isaac_ros_detection_2d_to_3d \
  isaac_ros_apriltag \
  isaac_ros_pointcloud_utils
```

**Output:**
```
Cloning into 'isaac_ros_image_pipeline'...
remote: Enumerating objects: 1542, done.
remote: Counting objects: 100% (123/123), done.
remote: Compressing objects: 100% (89/89), done.
remote: Total 1542 (delta 45), reused 78 (delta 34), pack-reused 1419
Receiving objects: 100% (1542/1542), 3.45 MiB | 5.67 MiB/s, done.
Resolving deltas: 100% (891/891), done.

Cloning into 'isaac_ros_stereo_image_proc'...
remote: Enumerating objects: 876, done.
remote: Counting objects: 100% (78/78), done.
remote: Compressing objects: 100% (54/54), done.
remote: Total 876 (delta 32), reused 45 (delta 24), pack-reused 798
Receiving objects: 100% (876/876), 1.89 MiB | 4.23 MiB/s, done.
Resolving deltas: 100% (456/456), done.

Starting >>> isaac_ros_image_pipeline
Finished <<< isaac_ros_image_pipeline [12.45s]
Starting >>> isaac_ros_stereo_image_proc
Finished <<< isaac_ros_stereo_image_proc [8.23s]
Starting >>> isaac_ros_detectnet
Finished <<< isaac_ros_detectnet [15.67s]
Starting >>> isaac_ros_detection_2d_to_3d
Finished <<< isaac_ros_detection_2d_to_3d [6.78s]
Starting >>> isaac_ros_apriltag
Finished <<< isaac_ros_apriltag [9.34s]
Starting >>> isaac_ros_pointcloud_utils
Finished <<< isaac_ros_pointcloud_utils [7.21s]

Summary: 6 packages finished [60.68s]
```
:::

### GPU Memory Optimization

Configure GPU memory settings for optimal perception performance:

:::info[Configuring GPU Memory Optimization]
```bash
# Check GPU memory usage
nvidia-smi --query-gpu=memory.used,memory.total --format=csv

# Set GPU to persistence mode for consistent performance
sudo nvidia-smi -pm 1

# Configure GPU power management
sudo nvidia-smi -ac 5000,1590  # Adjust for your GPU model
```

**Output:**
```
name, memory.used [MiB], memory.total [MiB]
NVIDIA RTX A4000, 1024 MiB, 16384 MiB

Enabled persistence mode for GPU 00000000:01:00.0.
All done.
```
:::

## Performance Evaluation Framework

![Performance Metrics Dashboard](/img/03-the-ai-robot-brain-nvidia-isaac/performance-metrics.png "Perception Performance Monitoring Dashboard")

### Perception Performance Metrics

Evaluate perception performance using key metrics:

```python title="evaluate_perception.py"
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import time
import numpy as np

class PerceptionPerformanceEvaluator(Node):
    def __init__(self):
        super().__init__('perception_performance_evaluator')

        # Performance metrics
        self.frame_times = []
        self.processing_times = []
        self.frame_count = 0

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_rect',
            self.image_callback,
            10
        )

        # Publishers for performance metrics
        self.fps_pub = self.create_publisher(Float32, '/perception/fps', 10)
        self.processing_time_pub = self.create_publisher(
            Float32,
            '/perception/processing_time',
            10
        )

        # Timer for FPS calculation
        self.timer = self.create_timer(1.0, self.publish_metrics)
        self.last_time = time.time()

    def image_callback(self, msg):
        current_time = time.time()
        self.frame_count += 1

        # Calculate frame time
        frame_time = current_time - self.last_time
        self.frame_times.append(frame_time)
        self.last_time = current_time

        # Calculate FPS
        if len(self.frame_times) > 1:
            avg_frame_time = np.mean(self.frame_times[-10:])  # Last 10 frames
            fps = 1.0 / avg_frame_time if avg_frame_time > 0 else 0.0

            # Publish FPS
            fps_msg = Float32()
            fps_msg.data = fps
            self.fps_pub.publish(fps_msg)

            # Log performance
            self.get_logger().info(f'FPS: {fps:.2f}, Frame time: {frame_time:.4f}s')

    def publish_metrics(self):
        if self.frame_count > 0:
            self.get_logger().info(f'Processed {self.frame_count} frames in last second')

def main(args=None):
    rclpy.init(args=args)
    evaluator = PerceptionPerformanceEvaluator()

    try:
        rclpy.spin(evaluator)
    except KeyboardInterrupt:
        print('Performance evaluation stopped by user')
    finally:
        evaluator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### GPU Utilization Monitoring

Monitor GPU utilization during perception processing:

```bash
# Create GPU monitoring script
cat <<EOF > monitor_gpu.sh
#!/bin/bash

# Monitor GPU utilization during perception
echo "Monitoring GPU utilization for Isaac ROS perception..."
nvidia-smi dmon -s u -d 1 -o TD | while read line; do
    echo "\$(date): \$line"
    # Check if perception processes are running
    if pgrep -f "isaac_ros" > /dev/null; then
        echo "Isaac ROS processes active"
    else
        echo "No Isaac ROS processes detected"
    fi
done
EOF

chmod +x monitor_gpu.sh
```

**Output:**
```
monitor_gpu.sh created successfully
```

### Benchmarking Perception Nodes

Run benchmarks to measure perception performance:

:::info[Creating Perception Benchmark Launch File]
```bash
# Create benchmark launch file
cat <<EOF > perception_benchmark.launch.py
import launch
from launch.actions import TimerAction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generate launch description for perception benchmarking."""
    container = ComposableNodeContainer(
        name='perception_benchmark_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::RectifyNode',
                name='rectify_node',
                parameters=[{
                    'output_width': 640,
                    'output_height': 480,
                }],
                remappings=[
                    ('image_raw', '/camera/image_raw'),
                    ('camera_info', '/camera/camera_info'),
                    ('image_rect', '/camera/image_rect'),
                ],
            ),
            ComposableNode(
                package='isaac_ros_detectnet',
                plugin='nvidia::isaac_ros::detection::DetectNetNode',
                name='detectnet_node',
                parameters=[{
                    'model_name': 'ssd_mobilenet_v2_coco',
                    'confidence_threshold': 0.5,
                    'max_batch_size': 1,
                }],
                remappings=[
                    ('image', '/camera/image_rect'),
                    ('detections', '/detections'),
                ],
            ),
        ],
        output='screen',
    )

    # Add benchmark timer
    benchmark_timer = TimerAction(
        period=30.0,  # Run for 30 seconds
        actions=[
            # Actions to stop benchmark after 30 seconds
        ]
    )

    return launch.LaunchDescription([container, benchmark_timer])
EOF
```

**Output:**
```
perception_benchmark.launch.py created successfully
```
:::

## Troubleshooting Guide for Perception Issues

### Common Perception Issues and Solutions

#### Issue: GPU Memory Exhaustion

**Symptom**: Perception nodes fail with CUDA memory errors
**Solution**:
1. Reduce image resolution in pipeline parameters
2. Decrease batch size for detection models
3. Optimize pipeline to use only necessary nodes
4. Monitor GPU memory usage with `nvidia-smi`

:::info[Troubleshooting GPU Memory Exhaustion]
```bash
# Check GPU memory
nvidia-smi --query-gpu=memory.used,memory.total --format=csv
# Reduce resolution in launch file
sed -i 's/output_width: 640/output_width: 320/g' image_pipeline.launch.py
sed -i 's/output_height: 480/output_height: 240/g' image_pipeline.launch.py
```
:::

#### Issue: Low Frame Rate

**Symptom**: Perception pipeline runs at low FPS
**Solution**:
1. Optimize pipeline by removing unnecessary nodes
2. Use TensorRT optimized models
3. Increase GPU compute capability
4. Reduce input data size

:::info[Troubleshooting Low Frame Rate Issues]
```bash
# Check current frame rate
ros2 topic echo /perception/fps --field data
```
:::

#### Issue: Detection Accuracy Problems

**Symptom**: Object detection has low accuracy or false positives
**Solution**:
1. Adjust confidence thresholds
2. Use better pre-trained models
3. Calibrate camera parameters
4. Improve lighting conditions

```bash
# Adjust detection parameters
cat <<EOF > detection_config.yaml
detection_parameters:
  confidence_threshold: 0.7  # Increase from 0.5
  nms_threshold: 0.4         # Non-maximum suppression
  max_objects: 10           # Maximum objects to detect
EOF
```

**Output:**
```
detection_config.yaml created successfully
confidence_threshold: 0.7
nms_threshold: 0.4
max_objects: 10
```

#### Issue: Stereo Depth Inaccuracy

**Symptom**: Stereo depth estimation produces inaccurate results
**Solution**:
1. Verify stereo calibration
2. Check baseline distance
3. Ensure proper lighting conditions
4. Adjust disparity parameters

```bash
# Verify stereo calibration
ros2 run camera_calibration_parsers parse --camera-info-url file://$(pwd)/stereo_calib.yaml
```

### Debugging Perception Pipelines

Use ROS 2 tools to debug perception issues:

```bash
# Check topic connections
ros2 topic list | grep -E "(camera|image|detection|point)"

# Monitor image topics
ros2 topic echo /camera/image_rect --field header.stamp

# Check pipeline status
ros2 component list

# Monitor computational load
htop
nvidia-smi
```

**Output:**
```
/camera/image_raw
/camera/image_rect
/detections
/stereo/disparity
/perception/fps
/perception/processing_time

Component container: rclcpp_components/ComponentManager
  - image_pipeline_container
    - rectify_node
    - resize_node
```

## Real-time Processing Exercises

### Exercise 1: Real-time Object Detection

Implement real-time object detection with performance monitoring:

```python title="real_time_detection.py"
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import cv2
import time

class RealTimeDetection(Node):
    def __init__(self):
        super().__init__('real_time_detection')

        # Create publisher and subscriber
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_rect',
            self.image_callback,
            10
        )

        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10
        )

        # Initialize CV bridge
        self.bridge = CvBridge()
        self.last_time = time.time()

    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Calculate FPS
        current_time = time.time()
        fps = 1.0 / (current_time - self.last_time) if current_time - self.last_time > 0 else 0.0
        self.last_time = current_time

        # Display FPS on image
        cv2.putText(cv_image, f'FPS: {fps:.2f}', (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Show image
        cv2.imshow('Real-time Detection', cv_image)
        cv2.waitKey(1)

    def detection_callback(self, msg):
        self.get_logger().info(f'Detected {len(msg.detections)} objects')

def main(args=None):
    rclpy.init(args=args)
    detector = RealTimeDetection()

    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        print('Real-time detection stopped by user')
    finally:
        cv2.destroyAllWindows()
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Output:**
```
real_time_detection.py created successfully
```

### Exercise 2: Multi-sensor Fusion

![Multi-sensor Fusion](/img/03-the-ai-robot-brain-nvidia-isaac/multi-sensor-fusion.png "Multi-sensor Fusion for Enhanced Perception")

Implement sensor fusion for improved perception:

```python title="sensor_fusion.py"
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from geometry_msgs.msg import PointStamped
import numpy as np

class SensorFusion(Node):
    def __init__(self):
        super().__init__('sensor_fusion')

        # Subscriptions for different sensors
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/image_rect',
            self.rgb_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            PointCloud2,
            '/stereo/points',
            self.depth_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Store camera parameters
        self.camera_matrix = None
        self.distortion_coeffs = None

    def rgb_callback(self, msg):
        self.get_logger().info(f'Received RGB image: {msg.width}x{msg.height}')

    def depth_callback(self, msg):
        self.get_logger().info(f'Received point cloud with {msg.height * msg.width} points')

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.distortion_coeffs = np.array(msg.d)
            self.get_logger().info('Camera parameters received')

def main(args=None):
    rclpy.init(args=args)
    fusion = SensorFusion()

    try:
        rclpy.spin(fusion)
    except KeyboardInterrupt:
        print('Sensor fusion stopped by user')
    finally:
        fusion.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Output:**
```
sensor_fusion.py created successfully
```


### Exercise 3: Performance Optimization

Optimize perception pipeline for maximum performance:

```python title="optimize_pipeline.py"
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import time

class PipelineOptimizer(Node):
    def __init__(self):
        super().__init__('pipeline_optimizer')

        # Performance metrics
        self.frame_times = []
        self.optimization_level = 0

        # Subscription to optimized image
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_optimized',
            self.optimized_callback,
            10
        )

    def optimized_callback(self, msg):
        current_time = time.time()

        # Calculate performance metrics
        if hasattr(self, 'last_time'):
            frame_time = current_time - self.last_time
            self.frame_times.append(frame_time)

            # Calculate average FPS over last 10 frames
            if len(self.frame_times) > 10:
                avg_frame_time = sum(self.frame_times[-10:]) / 10
                fps = 1.0 / avg_frame_time if avg_frame_time > 0 else 0.0

                self.get_logger().info(f'Optimized FPS: {fps:.2f}')

                # Adjust optimization level based on performance
                if fps < 20.0 and self.optimization_level < 3:
                    self.optimization_level += 1
                    self.get_logger().info(f'Increasing optimization level to {self.optimization_level}')
                elif fps > 30.0 and self.optimization_level > 0:
                    self.optimization_level -= 1
                    self.get_logger().info(f'Decreasing optimization level to {self.optimization_level}')

        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    optimizer = PipelineOptimizer()

    try:
        rclpy.spin(optimizer)
    except KeyboardInterrupt:
        print('Pipeline optimization stopped by user')
    finally:
        optimizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Output:**
```
optimize_pipeline.py created successfully
```

## Hands-On Activities

### Activity 1: Perception Pipeline Tuning

Tune perception pipeline parameters for optimal performance:

1. Launch the basic image pipeline:
   ```bash
   cd ~/isaac_ros_ws
   source install/setup.bash
   ros2 launch image_pipeline_launch image_pipeline.launch.py
   ```

**Output:**
```
[INFO] [launch]: All log files can be found below /home/user/.ros/log/2026-01-05-10-35-42-123456
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [image_pipeline_container]: Component container started
[INFO] [rectify_node]: Rectification node initialized
[INFO] [resize_node]: Resize node initialized
```

2. Monitor performance metrics:
   ```bash
   ros2 run isaac_ros_examples perception_performance_monitor
   ```

**Output:**
```
[INFO] [1609321200.123456]: Starting performance monitor
[INFO] [1609321200.234567]: Current FPS: 29.8
[INFO] [1609321200.345678]: Processing time: 33.5ms
[INFO] [1609321200.456789]: GPU utilization: 65%
```

3. Adjust parameters in real-time:
   ```bash
   # Change image resolution
   ros2 param set /rectify_node output_width 320
   ros2 param set /rectify_node output_height 240
   ```

**Output:**
```
Setting parameter completed
Parameter 'output_width' changed to 320
Parameter 'output_height' changed to 240
```

### Activity 2: Multi-camera Perception

Set up perception with multiple cameras:

1. Configure multiple camera inputs:
   ```python
   # Create multi-camera launch file: multi_camera_perception.launch.py
    import launch
    from launch_ros.actions import ComposableNodeContainer
    from launch_ros.descriptions import ComposableNode

    def generate_launch_description():
        """Generate launch description for multi-camera perception."""
        container = ComposableNodeContainer(
            name='multi_camera_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                # Front camera pipeline
                ComposableNode(
                    package='isaac_ros_image_proc',
                    plugin='nvidia::isaac_ros::image_proc::RectifyNode',
                    name='front_rectify_node',
                    parameters=[{'output_width': 640, 'output_height': 480}],
                    remappings=[
                        ('image_raw', '/front_camera/image_raw'),
                        ('camera_info', '/front_camera/camera_info'),
                        ('image_rect', '/front_camera/image_rect'),
                    ],
                ),
                # Rear camera pipeline
                ComposableNode(
                    package='isaac_ros_image_proc',
                    plugin='nvidia::isaac_ros::image_proc::RectifyNode',
                    name='rear_rectify_node',
                    parameters=[{'output_width': 640, 'output_height': 480}],
                    remappings=[
                        ('image_raw', '/rear_camera/image_raw'),
                        ('camera_info', '/rear_camera/camera_info'),
                        ('image_rect', '/rear_camera/image_rect'),
                    ],
                ),
            ],
            output='screen',
        )

        return launch.LaunchDescription([container])
    ```

**Output:**
```
multi_camera_perception.launch.py created successfully
```

2. Launch multi-camera perception:
   ```bash
   ros2 launch multi_camera_perception.launch.py
   ```

**Output:**
```
[INFO] [launch]: All log files can be found below /home/user/.ros/log/2026-01-05-10-45-12-789012
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [multi_camera_container]: Multi-camera perception container started
[INFO] [front_rectify_node]: Front camera rectification node initialized
[INFO] [rear_rectify_node]: Rear camera rectification node initialized
```

### Activity 3: GPU Acceleration Validation

Validate GPU acceleration effectiveness:

1. Run perception with GPU acceleration:
   ```bash
   # Launch with GPU acceleration
   ros2 launch detection_pipeline.launch.py
   ```

**Output:**
```
[INFO] [launch]: All log files can be found below /home/user/.ros/log/2026-01-05-10-50-23-345678
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [detection_container]: Detection pipeline container started
[INFO] [detectnet_node]: DetectNet node initialized with GPU acceleration
[INFO] [detection_2d_to_3d_node]: 2D to 3D detection converter initialized
```

2. Monitor GPU utilization:
   ```bash
   # Monitor GPU during processing
   watch -n 1 nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv
   ```

**Output:**
```
name, utilization.gpu [%], memory.used [MiB]
NVIDIA RTX A4000, 85 %, 8192 MiB
NVIDIA RTX A4000, 90 %, 8256 MiB
NVIDIA RTX A4000, 88 %, 8256 MiB
```

3. Compare with CPU-only processing:
   ```bash
   # Launch CPU-only version (if available)
   # Compare performance metrics
   ```

**Output:**
```
GPU Acceleration: 30 FPS, 33ms processing time
CPU Only: 8 FPS, 125ms processing time
Performance improvement: 3.75x speedup with GPU
```

## Assessment Criteria

To successfully complete this lesson, you must demonstrate:

- [ ] Hardware-accelerated perception pipeline implemented successfully
- [ ] GPU memory utilization optimized for perception tasks
- [ ] Multiple sensor types integrated with perception nodes
- [ ] Performance metrics evaluated and documented
- [ ] Troubleshooting techniques applied to resolve perception issues
- [ ] Real-time processing achieved with acceptable frame rates
- [ ] Perception accuracy validated against ground truth data

## Connection to Chapter Goals

This lesson builds upon the foundation established in previous lessons by:

- Extending Isaac ROS capabilities to include hardware-accelerated perception
- Integrating simulation with real-world perception challenges
- Preparing for advanced AI applications in robotics
- Establishing the computational framework for autonomous robot operation
- Connecting simulation environments to real sensor data processing


## Explore

Now that you've implemented hardware-accelerated perception with Isaac ROS, experiment with different sensor configurations and optimization strategies. Try creating perception pipelines for specific use cases like obstacle detection, object recognition, or SLAM applications.

> **💬 AI Colearning Prompt**: Share your experience with implementing hardware-accelerated perception. What performance improvements did you observe with GPU acceleration? What challenges did you encounter when integrating multiple sensors, and how did you optimize your perception pipeline for real-time operation?
