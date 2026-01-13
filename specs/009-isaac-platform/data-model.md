# Data Model: Isaac Platform for Physical AI and Humanoid Robotics

## Core Entities

### Isaac Simulation Environment
- **Name**: String identifier for the environment
- **Description**: Text description of the simulation environment
- **Physics Properties**: Configuration for gravity, friction, collision detection
- **Lighting Setup**: Parameters for photorealistic lighting
- **Objects**: Collection of 3D objects/models in the environment
- **Robot Models**: Collection of robot models placed in the environment
- **Sensor Configurations**: Setup for cameras, LiDAR, IMUs in the simulation

### Humanoid Robot Model
- **Model Name**: String identifier for the robot model
- **URDF Path**: Path to the Unified Robot Description Format file
- **Joint Configuration**: Definition of all robot joints and their properties
- **Link Properties**: Physical properties of robot links (mass, inertia, etc.)
- **Sensor Mounts**: Locations and types of sensors attached to the robot
- **Kinematic Properties**: Forward and inverse kinematics configurations
- **Actuator Specifications**: Properties of robot actuators and motors

### Perception Pipeline
- **Pipeline Name**: String identifier for the perception pipeline
- **Input Sources**: Configuration for sensor inputs (cameras, LiDAR, etc.)
- **Processing Nodes**: Collection of perception processing nodes
- **GPU Acceleration**: Configuration for hardware acceleration
- **Output Types**: Types of perception outputs (detections, classifications, etc.)
- **Performance Metrics**: Real-time performance measurements
- **Calibration Data**: Sensor calibration parameters

### Navigation Plan
- **Plan ID**: Unique identifier for the navigation plan
- **Start Pose**: Initial position and orientation of the robot
- **Goal Pose**: Target position and orientation for navigation
- **Path Waypoints**: Sequence of waypoints for the planned path
- **Footstep Plan**: For bipedal robots, the planned sequence of foot placements
- **Constraints**: Navigation constraints specific to humanoid robots
- **Execution Status**: Current status of plan execution

### Isaac Application
- **Application Name**: String identifier for the Isaac application
- **Components**: Collection of Isaac components in the application
- **Configuration**: Application-specific configuration parameters
- **Dependencies**: List of Isaac packages and ROS 2 packages required
- **Launch Files**: ROS 2 launch files for the application
- **Deployment Target**: Platform for application deployment (simulation, Jetson, etc.)

## Relationships

### Environment and Robot Model
- One Isaac Simulation Environment can contain multiple Humanoid Robot Models
- Each Humanoid Robot Model can exist in multiple environments (through instantiation)

### Perception Pipeline and Robot Model
- One Humanoid Robot Model can have multiple Perception Pipelines
- Each Perception Pipeline is associated with one primary robot model for sensor configuration

### Navigation Plan and Robot Model
- One Humanoid Robot Model can have multiple Navigation Plans (for different tasks)
- Each Navigation Plan is specific to one robot model's kinematic constraints

### Isaac Application and Components
- One Isaac Application can contain multiple Perception Pipelines
- One Isaac Application can manage multiple Navigation Plans
- One Isaac Application can interface with multiple Robot Models

## State Transitions

### Simulation Environment States
- **Created**: Environment has been initialized but not yet loaded
- **Loading**: Environment assets are being loaded
- **Running**: Simulation is actively running
- **Paused**: Simulation is temporarily stopped
- **Stopped**: Simulation has been stopped

### Perception Pipeline States
- **Initialized**: Pipeline has been created but not started
- **Starting**: Pipeline is being initialized with sensor data
- **Running**: Pipeline is actively processing sensor data
- **Fault**: Pipeline has encountered an error
- **Stopped**: Pipeline has been stopped

### Navigation Plan States
- **Planned**: Path has been computed but not yet executed
- **Executing**: Robot is following the planned path
- **Paused**: Navigation execution is temporarily stopped
- **Completed**: Robot has reached the goal pose
- **Failed**: Navigation could not be completed due to obstacles or errors

## Validation Rules

### Isaac Simulation Environment
- Name must be unique within the system
- Physics properties must be valid numerical values
- Environment must contain at least one robot model to be functional
- Lighting setup parameters must be within valid ranges

### Humanoid Robot Model
- URDF file must exist and be valid
- Joint configurations must not exceed physical limits
- Sensor mounts must be positioned correctly on the robot model
- Kinematic properties must be consistent with joint constraints

### Perception Pipeline
- Input sources must be properly configured and available
- GPU acceleration settings must be compatible with available hardware
- Performance metrics must meet minimum thresholds for real-time operation
- Calibration data must be current and valid

### Navigation Plan
- Start and goal poses must be within the same environment
- Path waypoints must form a continuous, valid path
- Footstep plans must maintain robot stability constraints
- Constraints must be physically achievable by the robot model

### Isaac Application
- All dependencies must be available before launch
- Configuration parameters must be valid for the target deployment platform
- Launch files must be properly formatted and executable
- Deployment target must be compatible with application requirements