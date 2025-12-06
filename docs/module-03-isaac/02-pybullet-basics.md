---
sidebar_position: 2
---

# Chapter 2: PyBullet Basics

## Introduction

PyBullet is a free, open-source physics simulation library that provides realistic physics without requiring expensive hardware. It's perfect for robotics simulation, machine learning, and research.

### Why PyBullet?

- **Free**: Completely open-source
- **Lightweight**: Works on normal laptops
- **Fast**: Efficient physics engine
- **Python-based**: Easy to use
- **ROS 2 Integration**: Full support
- **No GPU Required**: Runs on CPU

## Installation

### Simple Installation

```bash
# Install PyBullet
pip install pybullet

# Or with specific version
pip install pybullet==3.25

# Verify installation
python3 -c "import pybullet; print(pybullet.__version__)"
```

### Installation with ROS 2 Support

```bash
# Install PyBullet
pip install pybullet

# Install ROS 2 bridge (if needed)
pip install pybullet-ros2

# Or use standard ROS 2 packages
sudo apt install ros-humble-pybullet-ros
```

### System Requirements

- **Python**: 3.6 or higher
- **OS**: Linux, Windows, macOS
- **RAM**: 2GB minimum
- **CPU**: Any modern processor
- **GPU**: Optional (works without GPU)

## Basic PyBullet Usage

### Hello World Example

```python
#!/usr/bin/env python3
"""
PyBullet Hello World

Simple example showing basic PyBullet usage.
"""

import pybullet as p
import pybullet_data
import time

# Connect to physics server
physicsClient = p.connect(p.GUI)  # or p.DIRECT for no GUI

# Add search path for data files
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set gravity
p.setGravity(0, 0, -9.81)

# Load ground plane
planeId = p.loadURDF("plane.urdf")

# Load a simple box
boxId = p.loadURDF("cube.urdf", [0, 0, 1])

# Simulation loop
for i in range(10000):
    # Step simulation
    p.stepSimulation()
    
    # Get box position
    pos, orient = p.getBasePositionAndOrientation(boxId)
    print(f"Box position: {pos}")
    
    time.sleep(1.0/240.0)  # 240 Hz simulation

# Disconnect
p.disconnect()
```

## Loading Robots

### Loading URDF Files

```python
import pybullet as p
import pybullet_data

# Connect to physics server
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load robot from URDF
robotId = p.loadURDF("path/to/robot.urdf", [0, 0, 1])

# Get robot info
num_joints = p.getNumJoints(robotId)
print(f"Robot has {num_joints} joints")

# List all joints
for i in range(num_joints):
    joint_info = p.getJointInfo(robotId, i)
    print(f"Joint {i}: {joint_info[1]}")
```

### Loading from Model Library

PyBullet includes a model library:

```python
import pybullet as p
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load models from library
kuka = p.loadURDF("kuka_iiwa/model.urdf")
franka = p.loadURDF("franka_panda/panda.urdf")
humanoid = p.loadURDF("humanoid/humanoid.urdf")
```

### Loading SDF Files

```python
# Load SDF world file
objects = p.loadSDF("world.sdf")

# Load SDF model
model = p.loadSDF("model.sdf")
robotId = model[0]  # First object in SDF
```

## Physics Simulation

### Setting Up Physics

```python
import pybullet as p

# Connect
p.connect(p.GUI)

# Physics parameters
p.setGravity(0, 0, -9.81)
p.setTimeStep(1.0/240.0)  # 240 Hz
p.setPhysicsEngineParameter(
    numSolverIterations=10,
    numSubSteps=1
)
```

### Controlling Joints

```python
# Set joint position
p.setJointMotorControl2(
    bodyUniqueId=robotId,
    jointIndex=0,
    controlMode=p.POSITION_CONTROL,
    targetPosition=1.57  # radians
)

# Set joint velocity
p.setJointMotorControl2(
    bodyUniqueId=robotId,
    jointIndex=0,
    controlMode=p.VELOCITY_CONTROL,
    targetVelocity=1.0  # rad/s
)

# Set joint torque
p.setJointMotorControl2(
    bodyUniqueId=robotId,
    jointIndex=0,
    controlMode=p.TORQUE_CONTROL,
    force=10.0  # Nm
)
```

### Reading Joint States

```python
# Get joint state
joint_state = p.getJointState(robotId, joint_index)
position = joint_state[0]
velocity = joint_state[1]
reaction_forces = joint_state[2]
applied_torque = joint_state[3]

# Get all joint states
joint_states = p.getJointStates(robotId, range(num_joints))
```

### Applying Forces

```python
# Apply force to base
p.applyExternalForce(
    objectUniqueId=robotId,
    linkIndex=-1,  # -1 for base
    forceObj=[10, 0, 0],  # Force in x direction
    posObj=[0, 0, 0],
    flags=p.WORLD_FRAME
)

# Apply torque
p.applyExternalTorque(
    objectUniqueId=robotId,
    linkIndex=-1,
    torqueObj=[0, 0, 5],
    flags=p.WORLD_FRAME
)
```

## Complete Working Example

### Simple Robot Controller

```python
#!/usr/bin/env python3
"""
PyBullet Robot Controller

Complete example with robot loading, control, and simulation.
"""

import pybullet as p
import pybullet_data
import time
import math

# Connect to physics server
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Physics settings
p.setGravity(0, 0, -9.81)
p.setTimeStep(1.0/240.0)

# Load ground
planeId = p.loadURDF("plane.urdf")

# Load robot (example: Kuka arm)
robotId = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0.5])

# Get robot info
num_joints = p.getNumJoints(robotId)
print(f"Robot loaded with {num_joints} joints")

# Enable joint control
for i in range(num_joints):
    p.setJointMotorControl2(
        robotId,
        i,
        p.VELOCITY_CONTROL,
        targetVelocity=0,
        force=0
    )

# Simulation loop
for step in range(10000):
    # Control joints with sine wave
    for i in range(min(7, num_joints)):  # Control first 7 joints
        target_pos = 0.5 * math.sin(step * 0.01 + i * 0.5)
        p.setJointMotorControl2(
            robotId,
            i,
            p.POSITION_CONTROL,
            targetPosition=target_pos,
            maxVelocity=1.0
        )
    
    # Step simulation
    p.stepSimulation()
    
    # Get end effector position
    if num_joints > 0:
        link_state = p.getLinkState(robotId, num_joints - 1)
        end_effector_pos = link_state[0]
        
        if step % 100 == 0:
            print(f"End effector: {end_effector_pos}")
    
    time.sleep(1.0/240.0)

# Disconnect
p.disconnect()
```

## ROS 2 Integration

### PyBullet ROS 2 Node

```python
#!/usr/bin/env python3
"""
PyBullet ROS 2 Node

Integrates PyBullet simulation with ROS 2.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import pybullet as p
import pybullet_data


class PyBulletNode(Node):
    def __init__(self):
        super().__init__('pybullet_node')
        
        # Initialize PyBullet
        self.physics_client = p.connect(p.DIRECT)  # or p.GUI
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # Load robot
        self.robot_id = p.loadURDF("path/to/robot.urdf", [0, 0, 1])
        self.num_joints = p.getNumJoints(self.robot_id)
        
        # ROS 2 publishers
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # ROS 2 subscribers
        self.joint_cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/joint_commands',
            self.joint_command_callback,
            10
        )
        
        # Timer for simulation
        self.timer = self.create_timer(1.0/240.0, self.simulation_step)
        
        self.get_logger().info('PyBullet ROS 2 node started')
    
    def joint_command_callback(self, msg):
        """Handle joint commands from ROS 2."""
        for i, position in enumerate(msg.data[:self.num_joints]):
            p.setJointMotorControl2(
                self.robot_id,
                i,
                p.POSITION_CONTROL,
                targetPosition=position
            )
    
    def simulation_step(self):
        """Step simulation and publish joint states."""
        # Step simulation
        p.stepSimulation()
        
        # Get joint states
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        
        for i in range(self.num_joints):
            state = p.getJointState(self.robot_id, i)
            joint_state_msg.name.append(f"joint_{i}")
            joint_state_msg.position.append(state[0])
            joint_state_msg.velocity.append(state[1])
            joint_state_msg.effort.append(state[3])
        
        # Publish
        self.joint_state_pub.publish(joint_state_msg)
    
    def destroy_node(self):
        """Cleanup PyBullet."""
        p.disconnect(self.physics_client)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PyBulletNode()
    
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

### Using with Robot State Publisher

```python
# In your launch file
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package',
            executable='pybullet_node',
            name='pybullet_node'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        )
    ])
```

## Advanced Features

### Camera Rendering

```python
# Get camera image
width = 640
height = 480
view_matrix = p.computeViewMatrix(
    cameraEyePosition=[1, 1, 1],
    cameraTargetPosition=[0, 0, 0],
    cameraUpVector=[0, 0, 1]
)
projection_matrix = p.computeProjectionMatrixFOV(
    fov=60,
    aspect=width/height,
    nearVal=0.1,
    farVal=100.0
)

# Render
images = p.getCameraImage(
    width,
    height,
    view_matrix,
    projection_matrix
)

rgb_image = images[2]  # RGB image
depth_image = images[3]  # Depth image
```

### Collision Detection

```python
# Check collision between two objects
contact_points = p.getContactPoints(robotId, obstacleId)

if contact_points:
    print("Collision detected!")
    for point in contact_points:
        print(f"Contact force: {point[9]}")
```

### Debug Visualization

```python
# Draw line
p.addUserDebugLine(
    lineFromXYZ=[0, 0, 0],
    lineToXYZ=[1, 1, 1],
    lineColorRGB=[1, 0, 0],
    lineWidth=2
)

# Draw text
p.addUserDebugText(
    text="Hello",
    textPosition=[0, 0, 1],
    textColorRGB=[1, 1, 1]
)
```

## Best Practices

1. **Use appropriate time step**: 1/240 Hz is good default
2. **Disable unused joints**: Set velocity control with zero target
3. **Use DIRECT mode**: For headless/automated runs
4. **Clean up**: Always disconnect when done
5. **Optimize rendering**: Disable GUI when not needed

## Common Errors and Solutions

### Error 1: "Module not found"

```bash
# Solution: Install PyBullet
pip install pybullet
```

### Error 2: "URDF file not found"

```python
# Solution: Set search path
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# Or use absolute path
robotId = p.loadURDF("/full/path/to/robot.urdf")
```

### Error 3: "Robot falls through ground"

```python
# Solution: Check URDF has proper collision/inertial
# Or load ground plane first
planeId = p.loadURDF("plane.urdf")
```

## Next Steps

Continue learning:
- [Chapter 3: Computer Vision](03-computer-vision.md) - Process images with OpenCV
- [Chapter 4: SLAM Basics](04-slam-basics.md) - Build maps for free

