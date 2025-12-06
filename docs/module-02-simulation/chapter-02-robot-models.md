---
sidebar_position: 3
---

# Chapter 2: Robot Models

## Introduction

Creating accurate robot models is essential for realistic simulation. This chapter covers URDF and SDF formats for describing robots.

## URDF vs SDF

- **URDF**: ROS standard, simpler, good for basic robots
- **SDF**: Gazebo native, more features, better for complex models

## Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="0.4" ixy="0" ixz="0" iyy="0.4" iyz="0" izz="0.4"/>
    </inertial>
  </link>
</robot>
```

## Joints

```xml
<joint name="joint1" type="revolute">
  <parent link="base_link"/>
  <child link="link1"/>
  <origin xyz="0 0 0.5" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
</joint>
```

## Visual and Collision

- **Visual**: What you see (can be simplified)
- **Collision**: Physics interaction (should be simple for performance)

## Inertial Properties

```xml
<inertial>
  <mass value="1.0"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <inertia ixx="0.1" ixy="0" ixz="0"
           iyy="0.1" iyz="0" izz="0.1"/>
</inertial>
```

## Meshes

```xml
<visual>
  <geometry>
    <mesh filename="package://my_package/meshes/robot.stl"/>
  </geometry>
</visual>
```

## Gazebo-Specific Tags

```xml
<gazebo>
  <material>Gazebo/Blue</material>
  <static>false</static>
</gazebo>
```

## SDF Format

SDF provides more features:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="my_robot">
    <link name="base_link">
      <visual name="visual">
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
```

## Best Practices

- Keep collision geometries simple
- Use proper inertial properties
- Test models in simulation
- Document model parameters

## Tools

- **xacro**: Macros for URDF
- **URDF to SDF converter**: Convert formats
- **MeshLab**: Process 3D meshes

## Exercises

1. Create a simple mobile robot URDF
2. Add joints and links
3. Convert to SDF format

## Next Steps

Chapter 3 covers sensors and actuators in Gazebo simulation.

