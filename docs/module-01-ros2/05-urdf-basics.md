---
sidebar_position: 5
---

# Chapter 5: URDF Basics

## Introduction

URDF (Unified Robot Description Format) is an XML format for describing robot models. This chapter covers creating URDF files, understanding links and joints, and building a complete humanoid robot model.

## URDF Format Explained

URDF is an XML-based format that describes:
- **Links**: Physical parts of the robot (bodies, limbs)
- **Joints**: Connections between links (how parts move)
- **Visual**: What the robot looks like
- **Collision**: Physics interaction geometry
- **Inertial**: Mass and inertia properties

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links define physical parts -->
  <link name="base_link">
    <!-- Visual appearance -->
    <visual>...</visual>
    <!-- Collision geometry -->
    <collision>...</collision>
    <!-- Mass and inertia -->
    <inertial>...</inertial>
  </link>
  
  <!-- Joints connect links -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <!-- Joint properties -->
  </joint>
</robot>
```

## Creating a Humanoid Robot URDF

Let's build a complete humanoid robot step by step.

### Step 1: Base Structure

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  
  <!-- Base/Torso -->
  <link name="torso">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.4 0.3 0.6"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.4 0.3 0.6"/>
      </geometry>
    </collision>
    
    <inertial>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <mass value="10.0"/>
      <inertia ixx="0.4" ixy="0" ixz="0"
               iyy="0.4" iyz="0" izz="0.4"/>
    </inertial>
  </link>
  
</robot>
```

### Step 2: Head

```xml
  <!-- Head -->
  <link name="head">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
    
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  
  <!-- Neck Joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>
```

### Step 3: Arms

```xml
  <!-- Left Upper Arm -->
  <link name="left_upper_arm">
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </collision>
    
    <inertial>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0" ixz="0"
               iyy="0.02" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  
  <!-- Left Shoulder Joint -->
  <joint name="left_shoulder_pitch" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="20" velocity="2.0"/>
  </joint>
  
  <!-- Left Forearm -->
  <link name="left_forearm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
    
    <inertial>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.01" iyz="0" izz="0.005"/>
    </inertial>
  </link>
  
  <!-- Left Elbow Joint -->
  <joint name="left_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_forearm"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="3.14" effort="15" velocity="2.0"/>
  </joint>
  
  <!-- Left Hand -->
  <link name="left_hand">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.08 0.1 0.05"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.08 0.1 0.05"/>
      </geometry>
    </collision>
    
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0"
               iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  
  <!-- Left Wrist Joint -->
  <joint name="left_wrist" type="revolute">
    <parent link="left_forearm"/>
    <child link="left_hand"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="5" velocity="1.0"/>
  </joint>
```

### Step 4: Legs

```xml
  <!-- Left Thigh -->
  <link name="left_thigh">
    <visual>
      <origin xyz="0 0 -0.3" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.08" length="0.6"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 -0.3" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.08" length="0.6"/>
      </geometry>
    </collision>
    
    <inertial>
      <origin xyz="0 0 -0.3" rpy="0 0 0"/>
      <mass value="3.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0"
               iyy="0.1" iyz="0" izz="0.05"/>
    </inertial>
  </link>
  
  <!-- Left Hip Joint -->
  <joint name="left_hip_pitch" type="revolute">
    <parent link="torso"/>
    <child link="left_thigh"/>
    <origin xyz="0.1 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2.0"/>
  </joint>
  
  <!-- Left Shank -->
  <link name="left_shank">
    <visual>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.5"/>
      </geometry>
    </collision>
    
    <inertial>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.05" ixy="0" ixz="0"
               iyy="0.05" iyz="0" izz="0.02"/>
    </inertial>
  </link>
  
  <!-- Left Knee Joint -->
  <joint name="left_knee" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shank"/>
    <origin xyz="0 0 -0.6" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="3.14" effort="40" velocity="2.0"/>
  </joint>
  
  <!-- Left Foot -->
  <link name="left_foot">
    <visual>
      <origin xyz="0 0.1 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.25 0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0.1 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.25 0.05"/>
      </geometry>
    </collision>
    
    <inertial>
      <origin xyz="0 0.1 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  
  <!-- Left Ankle Joint -->
  <joint name="left_ankle" type="revolute">
    <parent link="left_shank"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="20" velocity="1.0"/>
  </joint>
```

### Step 5: Complete URDF with Right Side

The right arm and leg follow the same pattern. Here's the complete structure:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  
  <!-- Material definitions -->
  <material name="blue">
    <color rgba="0 0 1 1"/>
  </material>
  <material name="skin">
    <color rgba="1 0.8 0.6 1"/>
  </material>
  <material name="black">
    <color rgba="0 0 0 1"/>
  </material>
  
  <!-- Include all links and joints from above -->
  <!-- Torso, Head, Left Arm, Left Leg -->
  <!-- Right Arm and Right Leg (mirror of left) -->
  
</robot>
```

## Links: Visual, Collision, and Inertial

### Visual Element

Defines what the robot looks like:

```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="1 1 1"/>
    <!-- or <cylinder>, <sphere>, <mesh> -->
  </geometry>
  <material name="my_material">
    <color rgba="1 0 0 1"/>  <!-- Red -->
  </material>
</visual>
```

### Collision Element

Defines physics interaction (should be simpler than visual):

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="1 1 1"/>
  </geometry>
</collision>
```

**Best Practice**: Use simple collision geometries for performance.

### Inertial Element

Defines mass and inertia (required for physics):

```xml
<inertial>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <mass value="10.0"/>
  <inertia ixx="0.4" ixy="0" ixz="0"
           iyy="0.4" iyz="0" izz="0.4"/>
</inertial>
```

**Inertia Matrix**:
- `ixx, iyy, izz`: Moments of inertia
- `ixy, ixz, iyz`: Products of inertia (usually 0 for symmetric objects)

## Joints: Types and Properties

### Joint Types

1. **Fixed**: No movement
```xml
<joint name="fixed_joint" type="fixed">
  <parent link="base"/>
  <child link="attachment"/>
</joint>
```

2. **Revolute**: Rotation around one axis
```xml
<joint name="revolute_joint" type="revolute">
  <parent link="base"/>
  <child link="arm"/>
  <axis xyz="0 0 1"/>
  <limit lower="-3.14" upper="3.14" effort="10" velocity="1.0"/>
</joint>
```

3. **Continuous**: Unlimited rotation
```xml
<joint name="continuous_joint" type="continuous">
  <parent link="base"/>
  <child link="wheel"/>
  <axis xyz="0 0 1"/>
</joint>
```

4. **Prismatic**: Linear movement
```xml
<joint name="prismatic_joint" type="prismatic">
  <parent link="base"/>
  <child link="slider"/>
  <axis xyz="1 0 0"/>
  <limit lower="0" upper="1.0" effort="10" velocity="0.5"/>
</joint>
```

### Joint Limits

```xml
<limit lower="-1.57"    <!-- Minimum angle/position -->
      upper="1.57"      <!-- Maximum angle/position -->
      effort="10"      <!-- Maximum effort (torque/force) -->
      velocity="1.0"/>  <!-- Maximum velocity -->
```

## Using URDF in ROS 2

### Loading URDF

```python
from ament_index_python.packages import get_package_share_directory
import os

urdf_path = os.path.join(
    get_package_share_directory('my_robot_package'),
    'urdf',
    'humanoid.urdf'
)

with open(urdf_path, 'r') as f:
    robot_description = f.read()
```

### Robot State Publisher

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        )
    ])
```

### Visualizing in RViz2

```bash
# Launch robot state publisher
ros2 launch my_robot_package display.launch.py

# In another terminal, start RViz2
rviz2
```

## Common Errors and Solutions

### Error 1: "Link has no inertial element"

**Problem**: Physics simulation requires inertial properties.

**Solution**: Add inertial element to all links:
```xml
<inertial>
  <mass value="1.0"/>
  <inertia ixx="0.1" ixy="0" ixz="0"
           iyy="0.1" iyz="0" izz="0.1"/>
</inertial>
```

### Error 2: "Joint connects to non-existent link"

**Problem**: Joint references link that doesn't exist.

**Solution**: Check link names match exactly (case-sensitive).

### Error 3: "URDF file not found"

**Problem**: URDF path is incorrect.

**Solution**: Use `get_package_share_directory()` or absolute paths.

## Best Practices

1. **Keep collision simple**: Use boxes/cylinders, not complex meshes
2. **Set realistic masses**: Research actual component weights
3. **Calculate inertia properly**: Use CAD software or formulas
4. **Test incrementally**: Add links/joints one at a time
5. **Use xacro**: For reusable components (covered in advanced topics)

## Exercises

### Exercise 1: Simple Mobile Robot

Create a URDF for a simple differential drive robot with:
- Base link
- Two wheels
- Caster wheel

### Exercise 2: Add Sensors

Add camera and LiDAR links to the humanoid robot.

### Exercise 3: Joint Limits

Experiment with different joint limits and observe behavior in simulation.

## Next Steps

Continue learning:
- [Chapter 6: Launch Files](./06-launch-files.md) - Orchestrate multiple nodes
- Module 2: Simulation - Use URDF in Gazebo

