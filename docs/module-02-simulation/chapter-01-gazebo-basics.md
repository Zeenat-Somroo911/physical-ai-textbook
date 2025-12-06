---
sidebar_position: 2
---

# Chapter 1: Gazebo Basics

## Introduction

Gazebo is a powerful 3D physics simulator that enables you to test robots in realistic environments before deploying to hardware. This chapter introduces Gazebo's architecture and basic usage.

## What is Gazebo?

Gazebo provides:
- **Physics simulation**: Realistic dynamics and collisions
- **Sensor simulation**: Cameras, LiDAR, IMU, and more
- **3D rendering**: Visual feedback and debugging
- **Plugin system**: Extensible functionality
- **ROS integration**: Seamless connection with ROS2

## Installation

```bash
sudo apt-get update
sudo apt-get install ros-humble-gazebo-ros-pkgs
```

## Starting Gazebo

```bash
# Start empty world
gazebo

# Start with a specific world
gazebo worlds/empty.world

# With ROS2 integration
ros2 launch gazebo_ros gazebo.launch.py
```

## Gazebo Architecture

- **World**: Container for models, physics, and rendering
- **Models**: Robots, objects, and environments
- **Plugins**: Extend functionality (sensors, controllers)
- **Physics Engine**: ODE, Bullet, or Simbody
- **Rendering**: OGRE-based 3D graphics

## Basic Concepts

### World Files

World files (`.world`) define:
- Physics engine settings
- Scene properties (lighting, shadows)
- Models and their poses
- Plugin configurations

### Model Files

Models can be defined in:
- **URDF**: ROS standard format
- **SDF**: Gazebo's native format (more features)

### Launching with ROS2

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        )
    ])
```

## GUI Overview

Gazebo's GUI provides:
- **Scene**: 3D visualization
- **Left Panel**: Model tree and properties
- **Right Panel**: Simulation controls
- **Bottom Panel**: Log messages

## Basic Operations

- **Insert Model**: Drag from model library
- **Move Model**: Select and use translation tools
- **Inspect Properties**: Click on models
- **Play/Pause**: Control simulation

## Next Steps

In Chapter 2, we'll learn how to create robot models using URDF and SDF.

