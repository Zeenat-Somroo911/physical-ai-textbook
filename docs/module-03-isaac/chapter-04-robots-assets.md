---
sidebar_position: 5
---

# Chapter 4: Robots and Assets

## Introduction

This chapter covers loading, configuring, and working with robot models and assets in Isaac Sim.

## Loading Robot Models

### From USD Files

```python
from omni.isaac.core.utils.stage import add_reference_to_stage

# Load robot USD
add_reference_to_stage(
    usd_path="/Isaac/Robots/Franka/franka.usd",
    prim_path="/World/franka"
)
```

### From Asset Library

```python
from omni.isaac.core.robots import Robot

# Load robot
robot = Robot(
    prim_path="/World/franka",
    name="franka",
    position=[0, 0, 0]
)
```

## Robot Configuration

### Setting Initial Pose

```python
from omni.isaac.core.utils.prims import get_prim_at_path

robot_prim = get_prim_at_path("/World/franka")
robot_prim.GetAttribute("xformOp:translate").Set([0, 0, 1])
```

### Joint Configuration

```python
from omni.isaac.core.robots import Robot

robot = Robot(prim_path="/World/franka")

# Set joint positions
joint_positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
robot.set_joint_positions(joint_positions)
```

## Articulation

### Articulation API

```python
from omni.isaac.core.articulations import Articulation

articulation = Articulation(
    prim_path="/World/franka",
    name="franka"
)

# Get joint states
joint_positions = articulation.get_joint_positions()
joint_velocities = articulation.get_joint_velocities()
```

### Joint Control

```python
# Position control
articulation.set_joint_position_targets(target_positions)

# Velocity control
articulation.set_joint_velocity_targets(target_velocities)

# Effort control
articulation.set_joint_efforts(target_efforts)
```

## Asset Library

### Available Assets

Isaac Sim includes:
- **Robots**: Franka, UR10, Kuka, etc.
- **Environments**: Warehouses, offices, outdoor scenes
- **Objects**: Common household and industrial items

### Loading Assets

```python
from omni.isaac.core.utils.stage import add_reference_to_stage

# Load object from library
add_reference_to_stage(
    usd_path="/Isaac/Props/Blocks/block.usd",
    prim_path="/World/block"
)
```

## Custom Robot Import

### From URDF

```python
from omni.isaac.urdf import _urdf

# Import URDF
_urdf.import_urdf(
    "/path/to/robot.urdf",
    "/World/my_robot"
)
```

### From SDF

```python
from omni.isaac.sdof import _sdof

# Import SDF
_sdof.import_sdf(
    "/path/to/robot.sdf",
    "/World/my_robot"
)
```

## Robot Properties

### Mass and Inertia

```python
from pxr import UsdPhysics

# Get mass API
mass_api = UsdPhysics.MassAPI.Apply(link_prim)
mass_api.CreateMassAttr(1.0)
```

### Collision Properties

```python
from pxr import UsdPhysics

# Configure collision
collision_api = UsdPhysics.CollisionAPI.Apply(prim)
collision_api.CreateRestitutionAttr(0.5)
collision_api.CreateFrictionAttr(0.8)
```

## Multi-Robot Scenarios

```python
# Load multiple robots
robot1 = Robot(prim_path="/World/robot1", name="robot1")
robot2 = Robot(prim_path="/World/robot2", name="robot2")

# Position them
robot1.set_world_pose([0, 0, 0])
robot2.set_world_pose([2, 0, 0])
```

## Best Practices

- Use asset library when possible
- Verify robot properties
- Test joint limits
- Validate collision geometry
- Document custom imports

## Exercises

1. Load a robot from the asset library
2. Configure joint positions
3. Import a custom robot model

## Next Steps

Chapter 5 covers physics and rendering configuration.

