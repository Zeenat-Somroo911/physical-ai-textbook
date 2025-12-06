---
sidebar_position: 2
---

# Chapter 2: URDF vs SDF

## Introduction

When working with Gazebo, you'll encounter two main formats for describing robots: **URDF** (Unified Robot Description Format) and **SDF** (Simulation Description Format). Understanding the differences and when to use each is crucial for effective robot simulation.

## URDF vs SDF Comparison

### Key Differences

| Feature | URDF | SDF |
|---------|------|-----|
| **Primary Use** | ROS robot description | Gazebo simulation |
| **Format** | XML | XML |
| **Complexity** | Simpler | More features |
| **Nested Models** | Limited | Full support |
| **Plugins** | Basic | Extensive |
| **Physics** | Basic | Advanced |
| **Versioning** | Single version | Multiple versions (1.4, 1.5, 1.6) |

### When to Use URDF

- **ROS Integration**: When working primarily with ROS 2
- **Simple Robots**: For straightforward robot descriptions
- **Standard Workflows**: Following ROS conventions
- **Cross-Platform**: Need compatibility with other ROS tools

### When to Use SDF

- **Complex Simulations**: Advanced physics and rendering
- **Nested Models**: Robots with sub-assemblies
- **Gazebo-Specific Features**: Plugins, sensors, advanced materials
- **Performance**: Better optimization for simulation

## URDF Format Deep Dive

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links: Physical parts -->
  <link name="base_link">
    <visual>...</visual>
    <collision>...</collision>
    <inertial>...</inertial>
  </link>
  
  <!-- Joints: Connections -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
  </joint>
</robot>
```

### URDF Limitations in Gazebo

URDF has some limitations when used directly in Gazebo:

1. **No Nested Models**: Can't easily compose complex robots
2. **Limited Plugin Support**: Basic plugin integration
3. **Material Limitations**: Fewer material options
4. **Sensor Definition**: Requires Gazebo-specific tags

### Gazebo-Specific URDF Tags

To use URDF in Gazebo, you need Gazebo-specific extensions:

```xml
<robot name="my_robot">
  <link name="base_link">
    <!-- Standard URDF -->
    <visual>...</visual>
    
    <!-- Gazebo extensions -->
    <gazebo>
      <material>Gazebo/Blue</material>
      <static>false</static>
      <turnGravityOff>false</turnGravityOff>
    </gazebo>
  </link>
  
  <!-- Sensor plugin -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="camera1">
      <!-- Camera configuration -->
    </sensor>
  </gazebo>
</robot>
```

## SDF Format Deep Dive

### Basic SDF Structure

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="my_robot">
    <pose>0 0 0 0 0 0</pose>
    <static>false</static>
    
    <link name="base_link">
      <collision>...</collision>
      <visual>...</visual>
      <inertial>...</inertial>
    </link>
    
    <joint name="joint1" type="revolute">
      <parent>base_link</parent>
      <child>link1</child>
    </joint>
  </model>
</sdf>
```

### SDF Advantages

1. **Nested Models**: Include other models as sub-components
2. **Rich Materials**: Advanced material properties
3. **Better Physics**: More physics engine options
4. **Plugin System**: Extensive plugin support
5. **World Integration**: Can define entire worlds

### SDF Version Differences

```xml
<!-- SDF 1.4 (older) -->
<sdf version="1.4">
  <model name="robot">
    <!-- Different syntax -->
  </model>
</sdf>

<!-- SDF 1.6 (current) -->
<sdf version="1.6">
  <model name="robot">
    <!-- Modern syntax -->
  </model>
</sdf>
```

## Converting Between Formats

### URDF to SDF

Gazebo automatically converts URDF to SDF when loading:

```bash
# Automatic conversion
gazebo my_robot.urdf

# Manual conversion using gz command
gz sdf -p my_robot.urdf > my_robot.sdf
```

### SDF to URDF

Converting SDF to URDF is more complex and may lose information:

```python
# Using Python script
import xml.etree.ElementTree as ET

def sdf_to_urdf(sdf_file, urdf_file):
    # Parse SDF
    tree = ET.parse(sdf_file)
    root = tree.getroot()
    
    # Extract model information
    model = root.find('model')
    
    # Create URDF structure
    robot = ET.Element('robot', name=model.get('name'))
    
    # Convert links and joints
    # (Implementation details...)
    
    # Write URDF
    tree = ET.ElementTree(robot)
    tree.write(urdf_file)
```

**Note**: SDF to URDF conversion is lossy - some SDF features don't have URDF equivalents.

## Creating Custom Robot Models

### Method 1: Start with URDF

Best for ROS-integrated workflows:

```xml
<?xml version="1.0"?>
<robot name="custom_robot">
  <!-- Base link -->
  <link name="base">
    <visual>
      <geometry>
        <box size="0.4 0.3 0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.4 0.3 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0"
               iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  
  <!-- Add Gazebo extensions -->
  <gazebo>
    <plugin name="differential_drive" filename="libgazebo_ros_diff_drive.so">
      <!-- Plugin configuration -->
    </plugin>
  </gazebo>
</robot>
```

### Method 2: Start with SDF

Best for simulation-focused development:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="custom_robot">
    <link name="base">
      <collision name="collision">
        <geometry>
          <box>
            <size>0.4 0.3 0.2</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.4 0.3 0.2</size>
          </box>
        </geometry>
        <material>
          <script>
            <name>Gazebo/Blue</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.1</ixx>
          <iyy>0.1</iyy>
          <izz>0.1</izz>
        </inertia>
      </inertial>
    </link>
  </model>
</sdf>
```

## Complete Humanoid Example

Let's create a complete humanoid robot using SDF format (more suitable for complex models):

### Complete Humanoid SDF

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="humanoid_robot">
    <pose>0 0 1.0 0 0 0</pose>
    <static>false</static>
    
    <!-- Torso -->
    <link name="torso">
      <pose>0 0 0.3 0 0 0</pose>
      <collision name="torso_collision">
        <geometry>
          <box>
            <size>0.4 0.3 0.6</size>
          </box>
        </geometry>
      </collision>
      <visual name="torso_visual">
        <geometry>
          <box>
            <size>0.4 0.3 0.6</size>
          </box>
        </geometry>
        <material>
          <script>
            <name>Gazebo/Blue</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
      <inertial>
        <pose>0 0 0.3 0 0 0</pose>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.4</ixx>
          <iyy>0.4</iyy>
          <izz>0.4</izz>
        </inertia>
      </inertial>
    </link>
    
    <!-- Head -->
    <link name="head">
      <pose>0 0 0.6 0 0 0</pose>
      <collision name="head_collision">
        <geometry>
          <sphere>
            <radius>0.15</radius>
          </sphere>
        </geometry>
      </collision>
      <visual name="head_visual">
        <geometry>
          <sphere>
            <radius>0.15</radius>
          </sphere>
        </geometry>
        <material>
          <script>
            <name>Gazebo/Skin</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
      <inertial>
        <mass>2.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <iyy>0.01</iyy>
          <izz>0.01</izz>
        </inertia>
      </inertial>
    </link>
    
    <!-- Neck Joint -->
    <joint name="neck_joint" type="revolute">
      <parent>torso</parent>
      <child>head</child>
      <pose>0 0 0.3 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-1.57</lower>
          <upper>1.57</upper>
          <effort>10</effort>
          <velocity>1.0</velocity>
        </limit>
      </axis>
    </joint>
    
    <!-- Left Upper Arm -->
    <link name="left_upper_arm">
      <pose>0.2 0 0.2 0 0 0</pose>
      <collision name="left_upper_arm_collision">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.4</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="left_upper_arm_visual">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.4</length>
          </cylinder>
        </geometry>
        <material>
          <script>
            <name>Gazebo/Blue</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
      <inertial>
        <pose>0 0 -0.2 0 0 0</pose>
        <mass>1.5</mass>
        <inertia>
          <ixx>0.02</ixx>
          <iyy>0.02</iyy>
          <izz>0.01</izz>
        </inertia>
      </inertial>
    </link>
    
    <!-- Left Shoulder Joint -->
    <joint name="left_shoulder_pitch" type="revolute">
      <parent>torso</parent>
      <child>left_upper_arm</child>
      <pose>0.2 0 0.2 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-3.14</lower>
          <upper>3.14</upper>
          <effort>20</effort>
          <velocity>2.0</velocity>
        </limit>
      </axis>
    </joint>
    
    <!-- Left Forearm -->
    <link name="left_forearm">
      <pose>0 0 -0.4 0 0 0</pose>
      <collision name="left_forearm_collision">
        <geometry>
          <cylinder>
            <radius>0.04</radius>
            <length>0.3</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="left_forearm_visual">
        <geometry>
          <cylinder>
            <radius>0.04</radius>
            <length>0.3</length>
          </cylinder>
        </geometry>
        <material>
          <script>
            <name>Gazebo/Blue</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
      <inertial>
        <pose>0 0 -0.15 0 0 0</pose>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <iyy>0.01</iyy>
          <izz>0.005</izz>
        </inertia>
      </inertial>
    </link>
    
    <!-- Left Elbow Joint -->
    <joint name="left_elbow" type="revolute">
      <parent>left_upper_arm</parent>
      <child>left_forearm</child>
      <pose>0 0 -0.4 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>0</lower>
          <upper>3.14</upper>
          <effort>15</effort>
          <velocity>2.0</velocity>
        </limit>
      </axis>
    </joint>
    
    <!-- Left Hand -->
    <link name="left_hand">
      <pose>0 0 -0.3 0 0 0</pose>
      <collision name="left_hand_collision">
        <geometry>
          <box>
            <size>0.08 0.1 0.05</size>
          </box>
        </geometry>
      </collision>
      <visual name="left_hand_visual">
        <geometry>
          <box>
            <size>0.08 0.1 0.05</size>
          </box>
        </geometry>
        <material>
          <script>
            <name>Gazebo/Skin</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.001</ixx>
          <iyy>0.001</iyy>
          <izz>0.001</izz>
        </inertia>
      </inertial>
    </link>
    
    <!-- Left Wrist Joint -->
    <joint name="left_wrist" type="revolute">
      <parent>left_forearm</parent>
      <child>left_hand</child>
      <pose>0 0 -0.3 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-1.57</lower>
          <upper>1.57</upper>
          <effort>5</effort>
          <velocity>1.0</velocity>
        </limit>
      </axis>
    </joint>
    
    <!-- Legs follow similar pattern -->
    <!-- Left Thigh, Left Shank, Left Foot -->
    <!-- Right Arm and Leg (mirror of left) -->
    
  </model>
</sdf>
```

### Spawning Humanoid in Gazebo

```bash
# Spawn SDF model
gz model --spawn-file=humanoid_robot.sdf --model-name=humanoid

# Or with ROS 2
ros2 run gazebo_ros spawn_entity.py \
    -entity humanoid_robot \
    -file humanoid_robot.sdf \
    -x 0 -y 0 -z 1.0
```

## Best Practices

### URDF Best Practices

1. **Keep it Simple**: Use URDF for straightforward robot descriptions
2. **Add Gazebo Tags**: Use `<gazebo>` tags for simulation-specific features
3. **Test Incrementally**: Add components one at a time
4. **Use xacro**: For reusable components (advanced topic)

### SDF Best Practices

1. **Use Nested Models**: Compose complex robots from simpler parts
2. **Optimize Collision**: Keep collision geometries simple
3. **Material Scripts**: Use Gazebo material scripts for consistency
4. **Version Control**: Specify SDF version explicitly

## Common Pitfalls

### Pitfall 1: Mixing URDF and SDF Syntax

**Problem**: Using URDF syntax in SDF files or vice versa.

**Solution**: Learn the differences:
- URDF: `<box size="1 1 1"/>`
- SDF: `<box><size>1 1 1</size></box>`

### Pitfall 2: Missing Inertial Properties

**Problem**: Model doesn't respond to physics.

**Solution**: Always include inertial properties in both formats.

### Pitfall 3: Coordinate Frame Confusion

**Problem**: Robot spawns in wrong orientation.

**Solution**: Understand pose/origin conventions:
- URDF: `<origin xyz="x y z" rpy="roll pitch yaw"/>`
- SDF: `<pose>x y z roll pitch yaw</pose>`

## Next Steps

Continue learning:
- [Chapter 3: Physics Simulation](03-physics-simulation.md) - Master physics engines
- [Chapter 4: Sensor Simulation](04-sensor-simulation.md) - Add sensors to your robot

