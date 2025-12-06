---
sidebar_position: 4
---

# Chapter 3: Scene Composition

## Introduction

Scene composition is the foundation of Isaac Sim simulations. This chapter covers creating, organizing, and managing complex simulation scenes.

## Scene Structure

### Basic Scene

```python
from omni.isaac.kit import SimulationApp
import omni.usd

simulation_app = SimulationApp({"headless": False})

# Create a new stage (scene)
stage = omni.usd.get_context().get_stage()

# Your scene setup here

simulation_app.close()
```

## Adding Objects

### Ground Plane

```python
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_prim

# Add ground plane
ground_prim = create_prim(
    "/World/ground",
    "Xform",
    position=[0, 0, 0]
)
```

### Basic Shapes

```python
from omni.isaac.core.utils.prims import create_prim

# Create a cube
cube_prim = create_prim(
    "/World/cube",
    "Cube",
    position=[0, 0, 1],
    scale=[1, 1, 1]
)
```

## Lighting

### Environment Light

```python
from omni.isaac.core.utils.stage import add_reference_to_stage

# Add dome light
add_reference_to_stage(
    usd_path="/Isaac/Environments/Grid/default_environment.usd",
    prim_path="/World/defaultEnvironment"
)
```

### Custom Lighting

```python
from pxr import UsdLux

# Create directional light
light_prim = stage.DefinePrim("/World/sun", "DistantLight")
light = UsdLux.DistantLight(light_prim)
light.CreateIntensityAttr(1000)
light.CreateColorAttr((1.0, 1.0, 0.9))
```

## Camera Setup

```python
from omni.isaac.core.utils.prims import create_prim
from pxr import UsdGeom

# Create camera
camera_prim = create_prim("/World/camera", "Camera")
camera = UsdGeom.Camera(camera_prim)
camera.CreateFocalLengthAttr(24.0)
camera.CreateHorizontalApertureAttr(20.955)
```

## Material and Textures

### Applying Materials

```python
from omni.isaac.core.materials import PreviewSurface

# Create material
material = PreviewSurface(
    prim_path="/World/material",
    color=(0.8, 0.2, 0.2)
)

# Apply to object
cube_prim.GetAttribute("material:binding").Set(material.prim_path)
```

## Scene Organization

### Using Xforms

```python
# Group objects
xform_prim = create_prim("/World/robot_group", "Xform")

# Add children
create_prim("/World/robot_group/base", "Cube", parent=xform_prim)
create_prim("/World/robot_group/arm", "Cube", parent=xform_prim)
```

## Loading Pre-built Scenes

```python
from omni.isaac.core.utils.stage import add_reference_to_stage

# Load warehouse scene
add_reference_to_stage(
    usd_path="/Isaac/Environments/Simple_Warehouse/warehouse.usd",
    prim_path="/World/warehouse"
)
```

## Scene Hierarchy

Organize scenes with clear hierarchy:
- `/World`: Root level
- `/World/Environment`: Environment elements
- `/World/Robots`: Robot models
- `/World/Objects`: Dynamic objects

## Best Practices

- Use meaningful prim paths
- Organize with Xforms
- Reuse assets when possible
- Document scene structure
- Version control scene files

## Exercises

1. Create a simple scene with ground and objects
2. Add lighting and cameras
3. Organize with Xforms

## Next Steps

Chapter 4 covers working with robots and assets in Isaac Sim.

