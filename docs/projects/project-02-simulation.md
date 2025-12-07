---
sidebar_position: 3
---

# Project 2: Build Your Own Humanoid

## Project Overview

Design and build a complete humanoid robot in Gazebo simulation. This project covers URDF creation, physics simulation, sensor integration, and ROS 2 control.

### Learning Objectives

- Create a complete URDF model for a humanoid robot
- Understand robot kinematics and dynamics
- Integrate sensors (camera, IMU) into the model
- Control the robot using ROS 2
- Simulate realistic physics behavior

### Prerequisites

- Completed [Module 2: Simulation & Gazebo](../module-02-simulation/01-gazebo-intro.md)
- Understanding of URDF format
- Basic knowledge of robot kinematics
- Gazebo installed

## Project Requirements

### Functional Requirements

1. **Humanoid Structure**: 2 arms, 2 legs, head, torso
2. **Joints**: At least 12 DOF (degrees of freedom)
3. **Sensors**: Camera in head, IMU in torso
4. **Physics**: Realistic mass, inertia, collision geometry
5. **ROS 2 Control**: Control joints via topics/services

### Technical Requirements

- Valid URDF file
- Proper link/joint hierarchy
- Visual and collision meshes
- Gazebo plugins for sensors
- ROS 2 control interface

## Step-by-Step Implementation

### Step 1: Create ROS 2 Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake humanoid_robot \
    --dependencies rclpy std_msgs sensor_msgs geometry_msgs
cd ~/ros2_ws
colcon build --packages-select humanoid_robot
source install/setup.bash
```

### Step 2: Create URDF Model

Create `humanoid_robot/urdf/humanoid.urdf`:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">

  <!-- Base Link: Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.4 0.3 0.6"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.4 0.3 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.4" ixy="0.0" ixz="0.0"
               iyy="0.3" iyz="0.0"
               izz="0.5"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0"
               iyy="0.01" iyz="0.0"
               izz="0.01"/>
    </inertial>
  </link>

  <!-- Head Joint -->
  <joint name="head_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="3.14"/>
  </joint>

  <!-- Left Upper Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0"
               iyy="0.01" iyz="0.0"
               izz="0.01"/>
    </inertial>
  </link>

  <!-- Left Shoulder Joint -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0.15 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="20" velocity="3.14"/>
  </joint>

  <!-- Left Forearm -->
  <link name="left_forearm">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0"
               iyy="0.005" iyz="0.0"
               izz="0.005"/>
    </inertial>
  </link>

  <!-- Left Elbow Joint -->
  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_forearm"/>
    <origin xyz="0 0 -0.175" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="3.14" effort="15" velocity="3.14"/>
  </joint>

  <!-- Right Upper Arm -->
  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0"
               iyy="0.01" iyz="0.0"
               izz="0.01"/>
    </inertial>
  </link>

  <!-- Right Shoulder Joint -->
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="0.2 -0.15 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="20" velocity="3.14"/>
  </joint>

  <!-- Right Forearm -->
  <link name="right_forearm">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0"
               iyy="0.005" iyz="0.0"
               izz="0.005"/>
    </inertial>
  </link>

  <!-- Right Elbow Joint -->
  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_forearm"/>
    <origin xyz="0 0 -0.175" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="3.14" effort="15" velocity="3.14"/>
  </joint>

  <!-- Left Thigh -->
  <link name="left_thigh">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.4"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3.0"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0"
               iyy="0.04" iyz="0.0"
               izz="0.04"/>
    </inertial>
  </link>

  <!-- Left Hip Joint -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_thigh"/>
    <origin xyz="0.1 0.1 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="30" velocity="3.14"/>
  </joint>

  <!-- Left Shank -->
  <link name="left_shank">
    <visual>
      <geometry>
        <box size="0.08 0.08 0.35"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.08 0.08 0.35"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0"
               iyy="0.02" iyz="0.0"
               izz="0.02"/>
    </inertial>
  </link>

  <!-- Left Knee Joint -->
  <joint name="left_knee_joint" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shank"/>
    <origin xyz="0 0 -0.225" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="3.14" effort="25" velocity="3.14"/>
  </joint>

  <!-- Right Thigh -->
  <link name="right_thigh">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.4"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3.0"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0"
               iyy="0.04" iyz="0.0"
               izz="0.04"/>
    </inertial>
  </link>

  <!-- Right Hip Joint -->
  <joint name="right_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_thigh"/>
    <origin xyz="0.1 -0.1 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="30" velocity="3.14"/>
  </joint>

  <!-- Right Shank -->
  <link name="right_shank">
    <visual>
      <geometry>
        <box size="0.08 0.08 0.35"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.08 0.08 0.35"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0"
               iyy="0.02" iyz="0.0"
               izz="0.02"/>
    </inertial>
  </link>

  <!-- Right Knee Joint -->
  <joint name="right_knee_joint" type="revolute">
    <parent link="right_thigh"/>
    <child link="right_shank"/>
    <origin xyz="0 0 -0.225" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="3.14" effort="25" velocity="3.14"/>
  </joint>

  <!-- Gazebo Plugins -->
  <gazebo>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/humanoid</namespace>
        <remapping>image_raw:=camera/image_raw</remapping>
        <remapping>camera_info:=camera/camera_info</remapping>
      </ros>
      <camera_name>head_camera</camera_name>
      <frame_name>head_camera_frame</frame_name>
      <hack_baseline>0.07</hack_baseline>
      <min_depth>0.1</min_depth>
      <max_depth>10.0</max_depth>
      <update_rate>30</update_rate>
    </plugin>
  </gazebo>

  <gazebo reference="head">
    <sensor name="head_camera" type="camera">
      <update_rate>30</update_rate>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
        </image>
        <clip>
          <near>0.1</near>
          <far>10.0</far>
        </clip>
      </camera>
      <always_on>1</always_on>
      <visualize>1</visualize>
    </sensor>
  </gazebo>

  <gazebo reference="torso">
    <sensor name="imu" type="imu">
      <always_on>1</always_on>
      <update_rate>100</update_rate>
      <imu>
        <angular_velocity>
          <x>0.01</x>
          <y>0.01</y>
          <z>0.01</z>
        </angular_velocity>
        <linear_acceleration>
          <x>0.01</x>
          <y>0.01</y>
          <z>0.01</z>
        </linear_acceleration>
      </imu>
    </sensor>
  </gazebo>

  <!-- Material Definitions -->
  <material name="blue">
    <color rgba="0 0 1 1"/>
  </material>
  <material name="red">
    <color rgba="1 0 0 1"/>
  </material>
  <material name="green">
    <color rgba="0 1 0 1"/>
  </material>
  <material name="white">
    <color rgba="1 1 1 1"/>
  </material>

</robot>
```

### Step 3: Create Control Node

Create `humanoid_robot/humanoid_robot/control_node.py`:

```python
#!/usr/bin/env python3
"""
Humanoid Control Node

Controls humanoid robot joints via ROS 2.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math

class HumanoidControlNode(Node):
    def __init__(self):
        super().__init__('humanoid_control_node')
        
        # Publisher for joint commands
        self.joint_pub = self.create_publisher(
            JointState,
            '/humanoid/joint_commands',
            10
        )
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_callback)
        
        self.time = 0.0
        self.get_logger().info('Humanoid control node started')
    
    def control_callback(self):
        """Control loop: move robot joints."""
        self.time += 0.1
        
        # Create joint state message
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = [
            'head_joint',
            'left_shoulder_joint',
            'left_elbow_joint',
            'right_shoulder_joint',
            'right_elbow_joint',
            'left_hip_joint',
            'left_knee_joint',
            'right_hip_joint',
            'right_knee_joint'
        ]
        
        # Simple sinusoidal motion
        joint_msg.position = [
            math.sin(self.time) * 0.5,  # Head
            math.sin(self.time * 0.5) * 1.0,  # Left shoulder
            math.sin(self.time * 0.5 + math.pi/4) * 1.0,  # Left elbow
            math.sin(self.time * 0.5 + math.pi) * 1.0,  # Right shoulder
            math.sin(self.time * 0.5 + 3*math.pi/4) * 1.0,  # Right elbow
            math.sin(self.time * 0.3) * 0.5,  # Left hip
            math.sin(self.time * 0.3 + math.pi/2) * 0.8,  # Left knee
            math.sin(self.time * 0.3 + math.pi) * 0.5,  # Right hip
            math.sin(self.time * 0.3 + 3*math.pi/2) * 0.8,  # Right knee
        ]
        
        joint_msg.velocity = [0.0] * len(joint_msg.name)
        joint_msg.effort = [0.0] * len(joint_msg.name)
        
        self.joint_pub.publish(joint_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 4: Create Launch File

Create `humanoid_robot/launch/humanoid.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('humanoid_robot')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'humanoid.urdf')
    
    # Read URDF
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    
    return LaunchDescription([
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        # Gazebo
        Node(
            package='gazebo_ros',
            executable='gazebo',
            name='gazebo',
            output='screen',
            arguments=['--verbose']
        ),
        
        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=['-entity', 'humanoid', '-topic', 'robot_description'],
            output='screen'
        ),
        
        # Control node
        Node(
            package='humanoid_robot',
            executable='control_node',
            name='humanoid_control',
            output='screen'
        ),
    ])
```

## Testing Steps

### Test 1: URDF Validation

```bash
# Check URDF syntax
check_urdf ~/ros2_ws/src/humanoid_robot/urdf/humanoid.urdf

# View URDF tree
urdf_to_graphiz ~/ros2_ws/src/humanoid_robot/urdf/humanoid.urdf
```

**Expected Output**: No errors, valid URDF structure.

### Test 2: Launch in Gazebo

```bash
# Launch robot
ros2 launch humanoid_robot humanoid.launch.py
```

**Expected Output**: 
- Gazebo opens
- Humanoid robot appears
- Robot visible in simulation

### Test 3: Joint Control

```bash
# Check joint states
ros2 topic echo /joint_states

# Check control commands
ros2 topic echo /humanoid/joint_commands
```

**Expected Output**: Joint positions changing, robot moving.

### Test 4: Sensor Data

```bash
# Check camera
ros2 topic echo /humanoid/camera/image_raw

# Check IMU
ros2 topic echo /humanoid/imu
```

**Expected Output**: Camera images and IMU data publishing.

## Expected Outputs

### Gazebo Visualization

- Humanoid robot with:
  - Blue torso
  - White head
  - Red left arm
  - Green right arm
  - Blue legs
- Robot performing simple movements
- Camera view in head
- IMU data from torso

### ROS 2 Topics

```
/humanoid/camera/image_raw
/humanoid/camera/camera_info
/humanoid/imu
/joint_states
/humanoid/joint_commands
```

## Grading Rubric

### URDF Model (40 points)

- **Structure** (15 points): Proper link/joint hierarchy
- **Visual** (10 points): Visual geometry defined
- **Collision** (10 points): Collision geometry defined
- **Physics** (5 points): Mass and inertia properties

### Sensors (20 points)

- **Camera** (10 points): Camera sensor working
- **IMU** (10 points): IMU sensor working

### Control (20 points)

- **Joint Control** (10 points): Joints controllable
- **Movement** (10 points): Robot moves realistically

### ROS 2 Integration (20 points)

- **Topics** (10 points): All topics publishing
- **Launch File** (10 points): Launch file works correctly

## Extensions

### Extension 1: Walking Gait

- Implement walking pattern
- Balance control
- Stable locomotion

### Extension 2: Hand Control

- Add hands with fingers
- Grasping simulation
- Object manipulation

### Extension 3: Advanced Sensors

- LIDAR sensor
- Force/torque sensors
- Tactile sensors

## Troubleshooting

### Issue: Robot doesn't appear in Gazebo

**Solution**: Check URDF syntax, verify launch file paths.

### Issue: Joints not moving

**Solution**: Verify joint names match, check control node is running.

### Issue: Sensors not publishing

**Solution**: Check Gazebo plugins, verify sensor configuration.

## Next Steps

- [Project 3: Autonomous Navigation System](project-03-perception.md) - Perception project
- [Module 3: AI-Powered Perception](../module-03-isaac/01-free-alternatives.md) - Learn perception

---

**Congratulations!** You've built a complete humanoid robot in simulation! 🤖

