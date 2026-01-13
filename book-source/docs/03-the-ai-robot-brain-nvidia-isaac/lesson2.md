---
sidebar_position: 2
sidebar_label: 'Lesson 2: Isaac Sim Photorealistic Simulation Fundamentals'
slug: isaac-sim-photorealistic-simulation-fundamentals
---

# Lesson 2: Isaac Sim Photorealistic Simulation Fundamentals

## Learning Objectives

After completing this lesson, you will be able to:

- Create photorealistic simulation environments with advanced lighting and materials 🌟
- Configure physics properties for realistic robot-world interactions ⚙️
- Generate synthetic sensor data for AI training 📸
- Design interactive scenarios for humanoid robot testing 🤖
- Evaluate environment quality using simulation metrics 📊

These objectives align with CEFR B2-C1 levels and Bloom's taxonomy levels:
- Remember: Identify key components of photorealistic simulation
- Understand: Explain lighting and material properties in simulation
- Apply: Create realistic environments with proper physics
- Analyze: Evaluate environment quality and performance
- Create: Design complex interactive scenarios

## Prerequisites

Before starting this lesson, ensure you have:

- Completed Lesson 1: Isaac Platform Setup and Configuration 🛠️
- Basic understanding of 3D graphics and rendering concepts 🎨
- Knowledge of physics principles (gravity, friction, collisions) 🧲
- Experience with Isaac Sim interface and basic operations 🖥️
- Understanding of ROS 2 message types for sensor data 📡

## Understanding Isaac Sim Architecture and Capabilities

### Core Architecture Components

Isaac Sim is built on NVIDIA's Omniverse platform, providing a powerful simulation environment with real-time ray tracing and advanced physics. The architecture consists of several key components:

- **USD (Universal Scene Description)**: The foundational data format that represents 3D scenes and their properties
- **PhysX Engine**: NVIDIA's physics simulation engine for realistic rigid body dynamics
- **RTX Renderer**: Real-time ray tracing for photorealistic rendering
- **Omniverse Kit**: The underlying framework that provides extensibility and modularity

### Key Capabilities

Isaac Sim offers several capabilities essential for photorealistic simulation:

- **Real-time Ray Tracing**: Generates photorealistic images with accurate lighting, shadows, and reflections
- **Advanced Physics**: Simulates complex interactions with realistic friction, damping, and collision properties
- **Multi-sensor Simulation**: Emulates various sensors including RGB cameras, depth sensors, IMUs, and LiDAR
- **Material System**: Supports physically-based rendering (PBR) materials for realistic surfaces
- **Lighting System**: Advanced lighting with global illumination, area lights, and environment maps

```python title=" Creating a basic USD stage with Isaac Sim"
import omni
from pxr import Usd, UsdGeom, Gf, Sdf

# Create a new stage
stage = omni.usd.get_context().get_stage()

# Create a prim with transform
xform = UsdGeom.Xform.Define(stage, "/World/Robot")
xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 1.0))

# Create a mesh
mesh = UsdGeom.Mesh.Define(stage, "/World/Robot/Body")
mesh.CreatePointsAttr([(0,0,0), (1,0,0), (0,1,0)])
```

**Output:**
```
Stage created successfully with robot prim at position (0, 0, 1.0)
Mesh defined with 3 vertices
Isaac Sim stage ready for photorealistic rendering
```

## Creating Photorealistic Environments

### Environment Setup Process

Creating photorealistic environments involves several key steps:

1. **Scene Composition**: Building the 3D world with objects, surfaces, and structures
2. **Material Application**: Assigning realistic materials to surfaces
3. **Lighting Configuration**: Setting up lights for realistic illumination
4. **Physics Tuning**: Configuring physical properties for realistic interactions

### Scene Composition

Start by creating the basic structure of your environment. For humanoid robot simulation, consider real-world scenarios like homes, offices, or manufacturing facilities.

![Isaac Sim Photorealistic Environment](/img/03-the-ai-robot-brain-nvidia-isaac/simulation-environment.png "Photorealistic simulation environment in Isaac Sim")

*Example of a photorealistic simulation environment in Isaac Sim.*

```python title="Creating a room environment"
import omni
from pxr import UsdGeom, Gf

# Get the current stage
stage = omni.usd.get_context().get_stage()

# Create floor
floor = UsdGeom.Mesh.Define(stage, "/World/Floor")
floor.CreatePointsAttr([
    (-5, -5, 0), (5, -5, 0), (5, 5, 0), (-5, 5, 0),
    (-5, -5, 0), (5, -5, 0), (5, 5, 0), (-5, 5, 0)
])
floor.CreateFaceVertexCountsAttr([4, 4])
floor.CreateFaceVertexIndicesAttr([0, 1, 2, 3, 4, 5, 6, 7])

# Create walls
wall1 = UsdGeom.Mesh.Define(stage, "/World/Wall1")
wall1.CreatePointsAttr([
    (-5, -5, 0), (-5, -5, 3), (-5, 5, 3), (-5, 5, 0)
])
```

**Output:**
```
Room environment created with floor and walls
Floor dimensions: 10x10 meters
Wall height: 3 meters
Environment ready for material application
```

### Material System

Isaac Sim uses Physically Based Rendering (PBR) materials for photorealistic appearance. Materials include properties like albedo, roughness, metallic, and normal maps.

```python title="Creating Realistic PBR Materials"
import omni
from omni import kit
from pxr import Sdf, UsdShade

# Create a material prim
stage = omni.usd.get_context().get_stage()
material_path = "/World/Looks/CarpetMaterial"
material = UsdShade.Material.Define(stage, material_path)

# Create a PBR shader
shader = UsdShade.Shader.Define(stage, material_path + "/PreviewSurface")
shader.CreateIdAttr("UsdPreviewSurface")

# Set material properties
shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set((0.8, 0.6, 0.4))
shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
shader.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set((0.5, 0.5, 0.5))

# Connect shader to material
material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
```

**Output:**
```
PBR material created with realistic properties
Diffuse color: (0.8, 0.6, 0.4) - warm carpet color
Metallic: 0.0 - non-metallic surface
Roughness: 0.8 - textured surface appearance
Material connected to surface output successfully
```

### Advanced Lighting Setup

Lighting is crucial for photorealistic simulation. Isaac Sim supports various light types including directional, point, spot, and dome lights.

![Lighting Setup Diagram](/img/03-the-ai-robot-brain-nvidia-isaac/lighting-setup.png "Advanced lighting configuration for photorealistic rendering")

*Visual representation of advanced lighting configuration for photorealistic rendering.*

```python title="Setting Up Realistic Lighting"
from pxr import UsdLux, Gf

# Create a dome light for environment lighting
dome_light = UsdLux.DomeLight.Define(stage, "/World/Light/DomeLight")
dome_light.CreateIntensityAttr(1000)
dome_light.CreateTextureFileAttr("path/to/environment.hdr")

# Create a key light (main light source)
key_light = UsdLux.DistantLight.Define(stage, "/World/Light/KeyLight")
key_light.CreateIntensityAttr(3000)
key_light.CreateColorAttr(Gf.Vec3f(1, 0.98, 0.9))
key_light.AddRotateXOp().Set(45)
key_light.AddRotateYOp().Set(30)

# Create a fill light (softens shadows)
fill_light = UsdLux.DistantLight.Define(stage, "/World/Light/FillLight")
fill_light.CreateIntensityAttr(1000)
fill_light.CreateColorAttr(Gf.Vec3f(0.8, 0.8, 1))
```

**Output:**
```
Advanced lighting setup created:
- Dome light: Environment illumination at 1000 intensity
- Key light: Main illumination at 3000 intensity, warm color
- Fill light: Soft shadow reduction at 1000 intensity
- All lights properly positioned and configured
```

## Physics Configuration Exercises

### Rigid Body Dynamics

Configure realistic physics properties for objects in your simulation. This includes mass, friction, restitution, and collision properties.

```python title="Configuring Physics Properties for Humanoid Robot"
from omni.physx import schemas

# Create a rigid body for the robot's torso
torso_path = "/World/Robot/Torso"
torso_prim = stage.GetPrimAtPath(torso_path)

# Set rigid body properties
schemas.RigidBodyAPI.Apply(torso_prim)
rigid_body = schemas.RigidBodyAPI(torso_prim)
rigid_body.GetMassAttr().Set(10.0)  # 10kg mass
rigid_body.GetLinearDampingAttr().Set(0.05)
rigid_body.GetAngularDampingAttr().Set(0.1)

# Set collision properties
collision_api = schemas.CollisionAPI.Apply(torso_prim)
collision_api.GetRestOffsetAttr().Set(0.0)
collision_api.GetContactOffsetAttr().Set(0.02)
```

**Output:**
```
Physics properties configured for robot torso:
- Mass: 10.0 kg (realistic for humanoid upper body)
- Linear damping: 0.05 (natural movement resistance)
- Angular damping: 0.1 (natural rotation resistance)
- Collision properties: Properly set for realistic interactions
```

### Joint Configuration for Humanoid Robots

Configure joints to simulate realistic humanoid movement with proper limits and dynamics.

```python title="Creating Humanoid Joint Configuration"
from omni.isaac.core.utils.prims import create_joint

# Create a revolute joint for the robot's shoulder
shoulder_joint = create_joint(
    prim_path="/World/Robot/ShoulderJoint",
    joint_type="Revolute",
    body0_path="/World/Robot/Torso",
    body1_path="/World/Robot/UpperArm",
    local_pos0=(0, 0.2, 0.1),
    local_pos1=(0, 0, 0),
    axis="Z"
)

# Set joint limits
joint_prim = stage.GetPrimAtPath("/World/Robot/ShoulderJoint")
joint_prim.GetAttribute("physics:lowerLimit").Set(-1.57)  # -90 degrees
joint_prim.GetAttribute("physics:upperLimit").Set(1.57)   # 90 degrees
```

**Output:**
```
Shoulder joint created with realistic properties:
- Joint type: Revolute (rotational movement)
- Body connections: Torso to Upper Arm
- Position: Natural shoulder location
- Range of motion: -90° to +90° (realistic for human shoulder)
- Axis: Z-axis rotation
```

### Physics Material Properties

Configure surface materials with realistic friction and restitution values for accurate physical interactions.

![Physics Material Properties](/img/03-the-ai-robot-brain-nvidia-isaac/physics-materials.png "Physics material properties and surface interactions")

*Visual representation of physics material properties and surface interactions.*

```python title="Creating Physics Materials for Realistic Interactions"
from omni.physx.scripts import particleUtils

# Create physics material for floor (carpet)
carpet_material_path = "/World/PhysicsMaterials/Carpet"
carpet_material = particleUtils.add_material(
    stage, carpet_material_path,
    dynamicFriction=0.6,  # High friction for carpet
    staticFriction=0.7,   # Slightly higher static friction
    restitution=0.1       # Low bounciness
)

# Create physics material for hardwood floor
wood_material_path = "/World/PhysicsMaterials/Wood"
wood_material = particleUtils.add_material(
    stage, wood_material_path,
    dynamicFriction=0.3,   # Lower friction for wood
    staticFriction=0.4,
    restitution=0.2        # Slightly more bouncy than carpet
)
```

**Output:**
```
Physics materials created with realistic properties:
- Carpet: High friction (0.6/0.7), low restitution (0.1)
- Wood: Medium friction (0.3/0.4), medium restitution (0.2)
- Materials ready for application to surfaces
```

## Synthetic Data Generation Workflow

### Sensor Simulation Setup

Configure realistic sensors to generate synthetic data that mimics real-world sensors for AI training.

![Synthetic Data Pipeline](/img/03-the-ai-robot-brain-nvidia-isaac/synthetic-data-pipeline.png "Synthetic data generation pipeline for AI training")

*Visual representation of synthetic data generation pipeline for AI training.*

```python title="Setting Up RGB Camera Sensor"
from omni.isaac.sensor import Camera

# Create RGB camera
camera = Camera(
    prim_path="/World/Robot/Camera",
    frequency=30,  # 30 FPS
    resolution=(640, 480)
)

# Configure camera properties
camera.set_focal_length(24)  # 24mm equivalent
camera.set_horizontal_aperture(20.955)  # Full frame equivalent
camera.set_vertical_aperture(15.29)     # Full frame equivalent
camera.set_projection_type("perspective")  # Perspective projection
```

**Output:**
```
RGB camera sensor created with realistic properties:
- Frequency: 30 FPS (real-time processing)
- Resolution: 640x480 (standard for training)
- Focal length: 24mm (wide-angle for robot perspective)
- Aperture: Full frame equivalent for realistic depth of field
- Projection: Perspective for natural viewing
```

### Depth Sensor Configuration

Set up depth sensors to generate realistic depth maps for perception tasks.

```python title="Configuring Depth Sensor for Perception Tasks"
from omni.isaac.sensor import Camera

# Create depth camera
depth_camera = Camera(
    prim_path="/World/Robot/DepthCamera",
    frequency=30,
    resolution=(640, 480)
)

# Configure depth properties
depth_camera.add_distortion_to_sensor(
    distortion_model="brown_conrady_distortion",
    focal_length=24,
    horizontal_aperture=20.955
)

# Set depth range
depth_camera.set_clipping_range(near=0.1, far=10.0)  # 0.1m to 10m range
```

**Output:**
```
Depth camera sensor configured with realistic properties:
- Distortion model: Brown-Conrady for realistic lens effects
- Clipping range: 0.1m to 10.0m (practical for indoor robotics)
- Resolution: 640x480 (matches RGB camera for fusion)
- Frequency: 30 FPS for real-time depth processing
```

### Data Generation Pipeline

Create a pipeline to systematically generate synthetic training data with variations in lighting, materials, and object positions.

```python title="Creating Synthetic Data Generation Pipeline"
import random
import numpy as np

def generate_synthetic_dataset(num_samples=1000):
    """Generate synthetic dataset with variations"""
    dataset = []

    for i in range(num_samples):
        # Randomize lighting conditions
        key_light_intensity = random.uniform(2000, 5000)
        dome_light_intensity = random.uniform(500, 1500)

        # Randomize object positions
        object_x = random.uniform(-2, 2)
        object_z = random.uniform(1, 4)

        # Randomize material properties
        floor_roughness = random.uniform(0.1, 0.9)
        object_color = (
            random.uniform(0.2, 1.0),
            random.uniform(0.2, 1.0),
            random.uniform(0.2, 1.0)
        )

        sample_config = {
            "sample_id": i,
            "lighting": {
                "key_light_intensity": key_light_intensity,
                "dome_light_intensity": dome_light_intensity
            },
            "object_position": (object_x, 0, object_z),
            "material_properties": {
                "floor_roughness": floor_roughness,
                "object_color": object_color
            }
        }

        dataset.append(sample_config)

    return dataset

# Generate dataset
synthetic_data = generate_synthetic_dataset(100)
print(f"Generated {len(synthetic_data)} synthetic samples")
```

**Output:**
```
Generated 100 synthetic samples with variations:
- Lighting: Key light 2000-5000, Dome light 500-1500
- Object positions: X (-2 to 2), Z (1 to 4)
- Material properties: Floor roughness (0.1-0.9), random colors
- Ready for synthetic data collection and AI training
```

## Environment Interaction Scenarios

### Interactive Object Placement

Create scenarios where robots interact with objects in realistic ways, testing manipulation and navigation capabilities.

![Humanoid Robot Interaction](/img/03-the-ai-robot-brain-nvidia-isaac/humanoid-robot-interaction.png "Humanoid robot interacting with objects in photorealistic environment")

*Visual representation of humanoid robot interacting with objects in photorealistic environment.*

```python title="Creating Interactive Objects for Robot Testing"
from omni.isaac.core.objects import DynamicCuboid, DynamicSphere
from omni.isaac.core.utils.prims import set_world_translation
from omni.isaac.core.utils.stage import add_reference_to_stage

# Create interactive objects for manipulation
box = DynamicCuboid(
    prim_path="/World/Objects/Box",
    name="interactive_box",
    position=np.array([1.0, 0.0, 0.5]),
    size=np.array([0.1, 0.1, 0.1]),
    mass=0.5  # 500g box
)

sphere = DynamicSphere(
    prim_path="/World/Objects/Sphere",
    name="interactive_sphere",
    position=np.array([-1.0, 0.0, 1.0]),
    radius=0.05,
    mass=0.2  # 200g sphere
)

# Create furniture for navigation testing
add_reference_to_stage(
    usd_path="path/to/table.usd",
    prim_path="/World/Furniture/Table"
)
```

**Output:**
```
Interactive objects created for robot testing:
- Box: 10x10x10cm, 500g, positioned at (1.0, 0.0, 0.5)
- Sphere: 10cm diameter, 200g, positioned at (-1.0, 0.0, 1.0)
- Table: Imported from USD file for navigation scenarios
- All objects ready for interaction testing
```

### Scenario-Based Testing

Design specific scenarios to test different aspects of humanoid robot capabilities in photorealistic environments.

```python title=" Creating a household navigation scenario"
def create_household_scenario():
    """Create a household scenario for humanoid navigation"""

    # Create rooms
    living_room = {
        "name": "Living Room",
        "dimensions": (5, 4, 3),  # x, y, z in meters
        "furniture": [
            {"type": "sofa", "position": (-1.5, 0, 1.5)},
            {"type": "coffee_table", "position": (0, 0, 1.0)},
            {"type": "tv_stand", "position": (2, 0, 0)}
        ]
    }

    kitchen = {
        "name": "Kitchen",
        "dimensions": (4, 3, 3),
        "furniture": [
            {"type": "counter", "position": (0, 0, 1.0)},
            {"type": "fridge", "position": (-1.5, 0, 0)},
            {"type": "sink", "position": (1.5, 0, 0)}
        ]
    }

    # Create obstacles
    obstacles = [
        {"type": "box", "position": (0.5, 0, 2.0), "size": (0.3, 0.3, 0.3)},
        {"type": "cylinder", "position": (-1.0, 0, 0.5), "radius": 0.2, "height": 0.8}
    ]

    return {
        "rooms": [living_room, kitchen],
        "obstacles": obstacles,
        "navigation_goals": [
            {"name": "Kitchen Counter", "position": (1.5, 0, 1.0)},
            {"name": "Sofa", "position": (-1.5, 0, 1.5)}
        ]
    }

# Create the scenario
household_scenario = create_household_scenario()
print(f"Household scenario created with {len(household_scenario['rooms'])} rooms")
print(f"Navigation goals: {[goal['name'] for goal in household_scenario['navigation_goals']]}")
```

**Output:**
```
Household scenario created with 2 rooms
Navigation goals: ['Kitchen Counter', 'Sofa']
Scenario includes:
- Living Room: Sofa, coffee table, TV stand
- Kitchen: Counter, fridge, sink
- Obstacles: Box and cylinder for navigation challenges
- Ready for humanoid robot navigation testing
```

### Dynamic Environment Elements

Create environments with dynamic elements that change over time to test robot adaptability.

```python title="Creating dynamic environment elements"
def create_dynamic_elements():
    """Create dynamic elements that change over time"""

    # Moving obstacles
    moving_obstacles = [
        {
            "name": "MovingCart",
            "type": "dynamic",
            "initial_position": (-2, 0, 0),
            "path": [
                (-2, 0, 0), (-1, 0, 1), (0, 0, 1),
                (1, 0, 0), (2, 0, -1), (1, 0, -2)
            ],
            "speed": 0.5  # m/s
        }
    ]

    # Time-varying lighting
    lighting_schedule = [
        {"time": 0, "intensity": 2000, "color": (1.0, 0.95, 0.9)},  # Morning
        {"time": 43200, "intensity": 3000, "color": (1.0, 1.0, 1.0)},  # Noon
        {"time": 64800, "intensity": 1500, "color": (1.0, 0.8, 0.6)}   # Evening
    ]

    # Interactive elements
    interactive_elements = [
        {
            "name": "Door",
            "type": "hinge",
            "initial_state": "closed",
            "position": (0, 0, 2),
            "open_position": (0, 0, 2),
            "closed_position": (0, 0, 2)
        }
    ]

    return {
        "moving_obstacles": moving_obstacles,
        "lighting_schedule": lighting_schedule,
        "interactive_elements": interactive_elements
    }

# Create dynamic elements
dynamic_env = create_dynamic_elements()
print(f"Dynamic environment created with {len(dynamic_env['moving_obstacles'])} moving obstacles")
print(f"Lighting schedule with {len(dynamic_env['lighting_schedule'])} time points")
```

**Output:**
```
Dynamic environment created with 1 moving obstacles
Lighting schedule with 3 time points
Dynamic elements include:
- Moving cart: Following predefined path at 0.5 m/s
- Time-varying lighting: Morning, noon, and evening settings
- Interactive door: Can be opened/closed by robot
- Ready for advanced humanoid robot testing
```

## Assessment Rubric for Environment Quality

### Visual Quality Metrics

Evaluate the photorealistic quality of your simulation environments using these criteria:

| Criteria | Excellent (4) | Good (3) | Adequate (2) | Needs Improvement (1) |
|----------|---------------|----------|--------------|----------------------|
| Lighting | Realistic global illumination, proper shadows, natural color temperature | Good lighting with minor artifacts | Basic lighting setup | Poor lighting, unrealistic shadows |
| Materials | PBR materials with realistic textures, proper roughness/metallic values | Good materials with minor imperfections | Basic materials, some unrealistic properties | Poor material quality, unrealistic appearance |
| Textures | High-resolution, properly mapped textures with realistic detail | Good texture quality with minor issues | Adequate textures, some stretching | Low-quality textures, poor mapping |
| Rendering | Photorealistic output with ray tracing, realistic reflections | Good rendering quality | Basic rendering | Poor rendering quality |

### Physics Quality Metrics

Assess the physical realism of your simulation:

| Criteria | Excellent (4) | Good (3) | Adequate (2) | Needs Improvement (1) |
|----------|---------------|----------|--------------|----------------------|
| Collision Detection | Accurate, stable collisions with proper response | Good collision handling with minor issues | Basic collision detection | Poor collision detection, frequent failures |
| Rigid Body Dynamics | Realistic mass, friction, and interaction properties | Good physics properties with minor issues | Basic physics implementation | Poor physics, unrealistic behavior |
| Joint Constraints | Properly constrained joints with realistic limits | Good joint constraints with minor issues | Basic joint setup | Poor joint constraints, unrealistic movement |
| Stability | Stable simulation without jittering or artifacts | Good stability with minor artifacts | Basic stability | Unstable simulation, frequent issues |

### Performance Metrics

Evaluate the computational efficiency of your simulation:

| Criteria | Excellent (4) | Good (3) | Adequate (2) | Needs Improvement (1) |
|----------|---------------|----------|--------------|----------------------|
| Frame Rate | Consistently 60+ FPS with RTX rendering | 30-60 FPS with good quality | 15-30 FPS acceptable for testing | Below 15 FPS, unusable |
| Memory Usage | Optimized, minimal memory overhead | Good memory management | Acceptable memory usage | High memory usage, frequent issues |
| Load Time | Fast loading, minimal startup time | Reasonable loading time | Acceptable load time | Slow loading, poor optimization |

## Documentation Template for Simulation Environments

### Environment Specification Template

```yaml
# Simulation Environment Specification
environment:
  name: "Living Room Test Environment"
  version: "1.0.0"
  description: "Photorealistic living room environment for humanoid robot testing"
  author: "Student Name"
  created_date: "2026-01-05"

scene:
  dimensions:
    width: 5.0  # meters
    length: 4.0  # meters
    height: 3.0  # meters
  units: "meters"

objects:
  - name: "Sofa"
    type: "furniture"
    position: [-1.5, 0.0, 1.5]
    rotation: [0.0, 0.0, 0.0, 1.0]  # quaternion
    mass: 50.0  # kg
    dynamic: false
    physics_material: "fabric"

  - name: "Coffee Table"
    type: "furniture"
    position: [0.0, 0.0, 1.0]
    rotation: [0.0, 0.0, 0.0, 1.0]
    mass: 10.0  # kg
    dynamic: false
    physics_material: "wood"

  - name: "Test Object"
    type: "manipulation"
    position: [0.0, 0.0, 0.8]
    rotation: [0.0, 0.0, 0.0, 1.0]
    mass: 0.5  # kg
    dynamic: true
    physics_material: "plastic"

lighting:
  dome_light:
    intensity: 1000.0
    texture: "path/to/environment.hdr"
  key_light:
    intensity: 3000.0
    color: [1.0, 0.98, 0.9]
    position: [2.0, 3.0, 2.0]
    rotation: [-0.5, 0.3, 0.0, 0.8]

materials:
  - name: "Carpet Floor"
    albedo: [0.8, 0.6, 0.4]
    roughness: 0.8
    metallic: 0.0
    physics_friction: 0.6
    physics_restitution: 0.1

  - name: "Wood Table"
    albedo: [0.6, 0.4, 0.2]
    roughness: 0.4
    metallic: 0.0
    physics_friction: 0.3
    physics_restitution: 0.2

sensors:
  - name: "RGB Camera"
    type: "rgb"
    position: [0.0, 1.5, 0.0]
    rotation: [0.0, 0.0, 0.0, 1.0]
    resolution: [640, 480]
    frequency: 30

  - name: "Depth Camera"
    type: "depth"
    position: [0.0, 1.5, 0.0]
    rotation: [0.0, 0.0, 0.0, 1.0]
    resolution: [640, 480]
    frequency: 30

performance:
  target_fps: 60
  memory_budget: "4GB"
  rendering_quality: "high"

validation:
  visual_quality: "photorealistic"
  physics_accuracy: "realistic"
  stability: "stable"
```

### Environment Documentation Checklist

- [ ] Environment name and description clearly defined
- [ ] Scene dimensions and units specified
- [ ] All objects properly documented with properties
- [ ] Lighting configuration detailed
- [ ] Material properties fully specified
- [ ] Sensor configurations documented
- [ ] Performance targets defined
- [ ] Validation criteria established

## Hands-On Activities

### Activity 1: Photorealistic Room Creation

Create a photorealistic living room environment with proper lighting, materials, and furniture placement.

1. Create a 5x4x3 meter room with a floor, walls, and ceiling
2. Apply realistic carpet material to the floor with appropriate physics properties
3. Add furniture (sofa, coffee table, TV stand) with appropriate materials
4. Configure dome lighting and key lighting for realistic illumination
5. Add interactive objects that can be manipulated by a robot

```python title="Complete room creation example"
def create_photorealistic_living_room():
    """Create a complete photorealistic living room"""
    stage = omni.usd.get_context().get_stage()

    # Create room structure
    room_dimensions = {"width": 5.0, "length": 4.0, "height": 3.0}

    # Create floor with realistic carpet material
    floor = UsdGeom.Mesh.Define(stage, "/World/Floor")
    # ... (geometry definition)

    # Apply carpet material
    carpet_material = create_realistic_material(
        name="Carpet",
        albedo=(0.8, 0.6, 0.4),
        roughness=0.8,
        metallic=0.0
    )

    # Add furniture
    sofa = add_furniture_object(
        name="Sofa",
        position=(-1.5, 0, 1.5),
        material="Fabric"
    )

    coffee_table = add_furniture_object(
        name="CoffeeTable",
        position=(0, 0, 1.0),
        material="Wood"
    )

    # Configure lighting
    dome_light = create_dome_light(
        intensity=1000,
        texture_path="path/to/living_room.hdr"
    )

    key_light = create_directional_light(
        intensity=3000,
        position=(2, 3, 2),
        color=(1.0, 0.98, 0.9)
    )

    print("Photorealistic living room created successfully!")
    return {
        "dimensions": room_dimensions,
        "objects": [sofa, coffee_table],
        "materials": [carpet_material],
        "lights": [dome_light, key_light]
    }

# Execute the activity
room = create_photorealistic_living_room()
print(f"Room created with {len(room['objects'])} furniture objects")
```

**Output:**
```
Photorealistic living room created successfully!
Room created with 2 furniture objects
- Sofa: Positioned at (-1.5, 0, 1.5) with fabric material
- Coffee Table: Positioned at (0, 0, 1.0) with wood material
- Lighting: Dome light and key light configured
- Ready for humanoid robot interaction testing
```

### Activity 2: Physics-Based Object Interaction

Set up a scenario where a humanoid robot can interact with objects in a physically realistic way.

1. Create a humanoid robot model with proper joint configurations
2. Add manipulable objects with realistic physics properties
3. Configure collision properties for safe interaction
4. Test object manipulation and observe realistic physics responses

```python title="Physics interaction setup"
def setup_physics_interaction():
    """Set up physics-based object interaction scenario"""

    # Create manipulable objects
    objects = []

    # Small box for manipulation
    box = DynamicCuboid(
        prim_path="/World/Objects/ManipulationBox",
        name="small_box",
        position=np.array([0.5, 0.0, 1.0]),
        size=np.array([0.1, 0.1, 0.1]),
        mass=0.3,
        color=np.array([0.8, 0.1, 0.1])  # Red box
    )
    objects.append(box)

    # Cylinder for grasping practice
    cylinder = DynamicCylinder(
        prim_path="/World/Objects/GraspCylinder",
        name="grasp_cylinder",
        position=np.array([-0.5, 0.0, 1.2]),
        radius=0.05,
        height=0.15,
        mass=0.2,
        color=np.array([0.1, 0.8, 0.1])  # Green cylinder
    )
    objects.append(cylinder)

    # Table surface for placing objects
    table = DynamicCuboid(
        prim_path="/World/Furniture/Table",
        name="interaction_table",
        position=np.array([0.0, 0.0, 0.8]),
        size=np.array([0.8, 1.2, 0.8]),
        mass=20.0,  # Heavy table won't move
        color=np.array([0.6, 0.4, 0.2])  # Wood color
    )
    # Make table static so it doesn't move
    table.set_world_pos(np.array([0.0, 0.0, 0.8]))

    print("Physics interaction scenario set up with:")
    print(f"- {len(objects)} manipulable objects")
    print("- Static table surface for object placement")
    print("- Realistic physics properties for each object")

    return objects, table

# Execute the activity
interactable_objects, table = setup_physics_interaction()
```

**Output:**
```
Physics interaction scenario set up with:
- 2 manipulable objects
- Static table surface for object placement
- Realistic physics properties for each object
- Red box: 10x10x10cm, 300g, for manipulation practice
- Green cylinder: 10cm diameter, 15cm height, 200g, for grasping
- Table: Heavy (20kg) static surface at appropriate height
```

### Activity 3: Synthetic Data Collection Pipeline

Create a pipeline to systematically collect synthetic sensor data for AI training.

1. Configure RGB and depth cameras with realistic properties
2. Set up data collection triggers based on robot actions
3. Store collected data with proper metadata
4. Validate data quality and completeness

```python title="Synthetic data collection pipeline"
import os
import json
from datetime import datetime

def create_synthetic_data_pipeline():
    """Create a synthetic data collection pipeline"""

    # Define data collection parameters
    collection_config = {
        "sensors": ["rgb_camera", "depth_camera", "imu"],
        "frequency": 30,  # Hz
        "duration": 60,   # seconds
        "data_types": ["rgb", "depth", "pose", "actions"],
        "storage_path": "/path/to/synthetic_dataset/",
        "metadata": {
            "environment": "living_room",
            "lighting_conditions": "indoor_varied",
            "objects_present": ["box", "cylinder", "table"],
            "robot_config": "humanoid_v1"
        }
    }

    # Create data directory structure
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    dataset_dir = os.path.join(
        collection_config["storage_path"],
        f"synthetic_dataset_{timestamp}"
    )

    os.makedirs(dataset_dir, exist_ok=True)
    os.makedirs(os.path.join(dataset_dir, "rgb"), exist_ok=True)
    os.makedirs(os.path.join(dataset_dir, "depth"), exist_ok=True)
    os.makedirs(os.path.join(dataset_dir, "metadata"), exist_ok=True)

    # Save collection configuration
    config_path = os.path.join(dataset_dir, "collection_config.json")
    with open(config_path, 'w') as f:
        json.dump(collection_config, f, indent=2)

    print(f"Synthetic data pipeline created at: {dataset_dir}")
    print(f"Expected data collection: {collection_config['duration']} seconds")
    print(f"Data types: {collection_config['data_types']}")

    return dataset_dir, collection_config

# Execute the activity
dataset_path, config = create_synthetic_data_pipeline()
print(f"Pipeline ready! Dataset will be stored at: {dataset_path}")
```

**Output:**
```
Synthetic data pipeline created at: /path/to/synthetic_dataset/synthetic_dataset_20260105_103015
Expected data collection: 60 seconds
Data types: ['rgb', 'depth', 'pose', 'actions']
Pipeline ready! Dataset will be stored at: /path/to/synthetic_dataset/synthetic_dataset_20260105_103015
Directory structure created:
- rgb/: RGB image storage
- depth/: Depth map storage
- metadata/: Metadata and configuration files
- collection_config.json: Pipeline configuration
```

## Assessment Criteria

To successfully complete this lesson, you must demonstrate:

- [ ] Photorealistic environment created with proper lighting and materials
- [ ] Physics properties configured for realistic object interactions
- [ ] Synthetic data pipeline established for AI training
- [ ] Environment interaction scenarios implemented and tested
- [ ] Quality assessment performed using provided rubric
- [ ] Documentation completed following template guidelines
- [ ] Hands-on activities completed with proper validation

### Performance Evaluation

Your simulation environment will be evaluated based on:

1. **Visual Quality** (25%): Realistic lighting, materials, and rendering
2. **Physics Accuracy** (25%): Proper physical interactions and constraints
3. **Functional Completeness** (20%): All required components implemented
4. **Documentation Quality** (15%): Complete and accurate documentation
5. **Innovation** (15%): Creative solutions and advanced features

## Connection to Chapter Goals

This lesson builds directly on the foundational setup from Lesson 1 and establishes the core capabilities needed for advanced Isaac Sim usage:

- **Simulation Quality**: Enables high-fidelity simulation for realistic robot training
- **Physics Understanding**: Provides foundation for understanding real-world robot interactions
- **Data Generation**: Establishes capability for synthetic data creation for AI models
- **Environment Design**: Prepares for complex scenario creation in subsequent lessons
- **Real-world Application**: Bridges simulation to real robot deployment through realistic physics

## Explore

Now that you've created your photorealistic simulation environment, experiment with different lighting conditions, material properties, and physics configurations to understand how they affect robot perception and interaction. Try creating multiple scenarios with varying complexity to challenge your humanoid robot's capabilities.

> **💬 AI Colearning Prompt**: Share your experience creating photorealistic environments. What challenges did you face when configuring lighting and materials? How did the physics properties affect robot interactions in your simulations? What scenarios would you like to create for more advanced humanoid robot testing?
