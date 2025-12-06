---
sidebar_position: 6
---

# Chapter 6: Practical Lab - Build Humanoid in Gazebo

## Lab Overview

This comprehensive lab guides you through building a complete humanoid robot simulation in Gazebo. By the end, you'll have a fully functional humanoid with sensors, ROS 2 integration, and Python control.

### Learning Objectives

- Create a complete humanoid robot URDF/SDF
- Add multiple sensors (camera, LiDAR, IMU)
- Integrate with ROS 2
- Control the robot from Python
- Visualize in RViz2
- Troubleshoot common issues

### Prerequisites

- ROS 2 Humble installed
- Gazebo 11 installed
- Basic Python knowledge
- Completion of previous chapters

### Estimated Time

4-6 hours

## Step 1: Project Setup

### Create Workspace

```bash
# Create workspace
mkdir -p ~/humanoid_ws/src
cd ~/humanoid_ws/src

# Create package
ros2 pkg create --build-type ament_python humanoid_robot \
    --dependencies rclpy std_msgs sensor_msgs geometry_msgs
```

### Directory Structure

```
humanoid_robot/
├── package.xml
├── setup.py
├── setup.cfg
├── humanoid_robot/
│   ├── __init__.py
│   ├── nodes/
│   │   ├── __init__.py
│   │   ├── controller.py
│   │   └── sensor_processor.py
│   └── launch/
│       └── humanoid.launch.py
├── urdf/
│   └── humanoid.urdf
├── worlds/
│   └── test_world.world
└── config/
    └── controllers.yaml
```

## Step 2: Create Humanoid URDF

Create `urdf/humanoid.urdf`:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  
  <!-- Material Definitions -->
  <material name="blue">
    <color rgba="0 0 1 1"/>
  </material>
  <material name="skin">
    <color rgba="1 0.8 0.6 1"/>
  </material>
  <material name="black">
    <color rgba="0 0 0 1"/>
  </material>
  
  <!-- Base/Torso -->
  <link name="torso">
    <visual>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <box size="0.4 0.3 0.6"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
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
  
  <!-- Head -->
  <link name="head">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="skin"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
    <inertial>
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
  
  <!-- Left Upper Arm -->
  <link name="left_upper_arm">
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <material name="blue"/>
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
      <material name="blue"/>
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
      <material name="skin"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.08 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
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
  
  <!-- Right Arm (mirror of left) -->
  <link name="right_upper_arm">
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <material name="blue"/>
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
  
  <joint name="right_shoulder_pitch" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.2 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="20" velocity="2.0"/>
  </joint>
  
  <!-- Add right forearm, hand, and joints similarly -->
  
  <!-- Left Thigh -->
  <link name="left_thigh">
    <visual>
      <origin xyz="0 0 -0.3" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.08" length="0.6"/>
      </geometry>
      <material name="blue"/>
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
      <material name="blue"/>
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
      <material name="black"/>
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
  
  <!-- Right Leg (mirror of left) -->
  <!-- Add right thigh, shank, foot, and joints similarly -->
  
  <!-- Camera Link -->
  <link name="camera_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0"
               iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  
  <joint name="camera_joint" type="fixed">
    <parent link="head"/>
    <child link="camera_link"/>
    <origin xyz="0.1 0 0" rpy="0 0 0"/>
  </joint>
  
  <!-- LiDAR Link -->
  <link name="lidar_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0" ixz="0"
               iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  
  <joint name="lidar_joint" type="fixed">
    <parent link="torso"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>
  
  <!-- IMU Link -->
  <link name="imu_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.02 0.02 0.02"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.0001" ixy="0" ixz="0"
               iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>
  
  <joint name="imu_joint" type="fixed">
    <parent link="torso"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  
  <!-- Gazebo Plugins -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="camera1">
      <update_rate>30</update_rate>
      <camera name="head">
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>/camera</namespace>
        </ros>
        <camera_name>camera</camera_name>
        <frame_name>camera_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
  
  <gazebo reference="lidar_link">
    <sensor type="ray" name="head_lidar">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>40</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.10</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="gazebo_ros_lidar_controller" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>/lidar</namespace>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>lidar_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
  
  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
        <ros>
          <namespace>/imu</namespace>
        </ros>
        <initial_orientation_as_reference>false</initial_orientation_as_reference>
      </plugin>
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>true</visualize>
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </z>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>
    </sensor>
  </gazebo>
  
</robot>
```

## Step 3: Create World File

Create `worlds/test_world.world`:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="test_world">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.81</gravity>
    </physics>
    
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>
    
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Obstacles for testing -->
    <model name="box1">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Step 4: Create Launch File

Create `humanoid_robot/launch/humanoid.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('humanoid_robot')
    
    # URDF file
    urdf_file = os.path.join(pkg_dir, 'urdf', 'humanoid.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # World file
    world_file = os.path.join(pkg_dir, 'worlds', 'test_world.world')
    
    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        
        # Spawn Robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'humanoid_robot', '-file', urdf_file, '-z', '1.0'],
            output='screen'
        ),
        
        # Joint State Publisher (for manual control)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        )
    ])
```

## Step 5: Create Python Controller

Create `humanoid_robot/humanoid_robot/nodes/controller.py`:

```python
#!/usr/bin/env python3
"""
Humanoid Robot Controller

Controls humanoid robot joints via ROS 2.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math
import time


class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')
        
        # Publisher for joint commands
        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_callback)
        
        # Joint state
        self.joint_positions = {
            'neck_joint': 0.0,
            'left_shoulder_pitch': 0.0,
            'left_elbow': 0.0,
            'right_shoulder_pitch': 0.0,
            'right_elbow': 0.0,
            'left_hip_pitch': 0.0,
            'left_knee': 0.0,
            'right_hip_pitch': 0.0,
            'right_knee': 0.0,
        }
        
        self.time = 0.0
        self.get_logger().info('Humanoid controller started')
    
    def control_callback(self):
        """Control loop - creates simple walking motion."""
        self.time += 0.1
        
        # Simple walking pattern
        self.joint_positions['left_hip_pitch'] = 0.3 * math.sin(self.time)
        self.joint_positions['right_hip_pitch'] = -0.3 * math.sin(self.time)
        self.joint_positions['left_knee'] = 0.5 * abs(math.sin(self.time))
        self.joint_positions['right_knee'] = 0.5 * abs(math.sin(self.time + math.pi))
        
        # Arm swing
        self.joint_positions['left_shoulder_pitch'] = 0.5 * math.sin(self.time)
        self.joint_positions['right_shoulder_pitch'] = -0.5 * math.sin(self.time)
        
        # Publish joint states
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_positions.keys())
        msg.position = list(self.joint_positions.values())
        
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 6: Build and Launch

```bash
# Build package
cd ~/humanoid_ws
colcon build --packages-select humanoid_robot
source install/setup.bash

# Launch simulation
ros2 launch humanoid_robot humanoid.launch.py

# In another terminal, start controller
ros2 run humanoid_robot controller

# In third terminal, start RViz2
rviz2
```

## Step 7: Visualize in RViz2

1. **Add RobotModel**: Display -> Add -> RobotModel
   - Robot Description: `robot_description`
   
2. **Add Camera**: Display -> Add -> Image
   - Image Topic: `/camera/image_raw`
   
3. **Add LiDAR**: Display -> Add -> LaserScan
   - Topic: `/lidar/scan`
   
4. **Add IMU**: Display -> Add -> Imu
   - Topic: `/imu/data`

## Troubleshooting Guide

### Issue 1: Robot Doesn't Spawn

**Symptoms**: No robot appears in Gazebo

**Solutions**:
- Check URDF syntax: `check_urdf urdf/humanoid.urdf`
- Verify file paths in launch file
- Check Gazebo console for errors
- Ensure all links have inertial properties

### Issue 2: Robot Falls Through Ground

**Symptoms**: Robot drops through ground plane

**Solutions**:
- Check collision geometries
- Verify inertial properties
- Ensure ground plane is included
- Check physics engine settings

### Issue 3: Sensors Not Publishing

**Symptoms**: No data on sensor topics

**Solutions**:
- Verify plugin filenames are correct
- Check frame names match
- Ensure sensors are properly attached
- Check Gazebo console for plugin errors

### Issue 4: Controller Not Working

**Symptoms**: Joints don't move

**Solutions**:
- Verify joint names match exactly
- Check joint limits
- Ensure joint_state_publisher is running
- Verify controller is publishing

### Issue 5: Slow Performance

**Symptoms**: Simulation runs slowly

**Solutions**:
- Reduce sensor update rates
- Simplify collision geometries
- Increase physics step size (carefully)
- Close unnecessary applications

## Testing Checklist

- [ ] Robot spawns correctly in Gazebo
- [ ] All links visible and properly connected
- [ ] Camera publishes images
- [ ] LiDAR publishes scans
- [ ] IMU publishes data
- [ ] Controller moves joints
- [ ] RViz2 displays all sensors
- [ ] Robot maintains balance (or falls realistically)
- [ ] No errors in console

## Advanced Exercises

### Exercise 1: Add More Sensors

Add additional sensors:
- Depth camera
- Force/torque sensors on feet
- Contact sensors

### Exercise 2: Implement Balance Control

Create a balance controller that keeps the robot upright.

### Exercise 3: Walking Gait

Implement a proper walking gait pattern.

### Exercise 4: Object Manipulation

Add grippers and implement object picking.

## Module 2 Summary

Congratulations! You've completed Module 2: Robot Simulation. You now understand:

- ✅ Gazebo installation and basics
- ✅ URDF vs SDF formats
- ✅ Physics simulation
- ✅ Sensor simulation
- ✅ Unity integration
- ✅ Complete robot simulation setup

## Next Steps

Continue your learning:
- **Module 3: AI-Powered Perception** - Computer vision and SLAM
- **Module 4: Vision-Language-Action** - Natural language control

Keep building amazing robots! 🤖

