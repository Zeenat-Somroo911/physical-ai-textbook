---
sidebar_position: 7
---

# Chapter 6: ROS Bridge

## Introduction

This chapter covers integrating Isaac Sim with ROS2 using the ROS Bridge extension, enabling seamless communication between simulation and ROS2 nodes.

## ROS Bridge Extension

### Installation

The ROS Bridge extension is included with Isaac Sim. Enable it in the extension manager or via code:

```python
from omni.isaac.core.utils.extensions import enable_extension

enable_extension("omni.isaac.ros2_bridge")
```

## Basic Setup

### Starting ROS Bridge

```python
from omni.isaac.core import SimulationContext
from omni.isaac.ros2_bridge import ROS2Bridge

# Initialize simulation
simulation_context = SimulationContext()

# Create ROS bridge
ros_bridge = ROS2Bridge()
ros_bridge.initialize()
```

## Publishing Topics

### Publishing Joint States

```python
from omni.isaac.ros2_bridge import ROS2Bridge
from sensor_msgs.msg import JointState

# Create publisher
bridge = ROS2Bridge()
publisher = bridge.create_publisher(JointState, "/joint_states")

# Publish joint states
joint_state = JointState()
joint_state.name = ["joint1", "joint2"]
joint_state.position = [0.5, 1.0]
publisher.publish(joint_state)
```

### Publishing Sensor Data

```python
from sensor_msgs.msg import Image, LaserScan

# Camera publisher
camera_pub = bridge.create_publisher(Image, "/camera/image_raw")

# LiDAR publisher
lidar_pub = bridge.create_publisher(LaserScan, "/scan")
```

## Subscribing to Topics

### Receiving Commands

```python
from geometry_msgs.msg import Twist

def cmd_vel_callback(msg):
    linear = msg.linear.x
    angular = msg.angular.z
    # Apply to robot
    robot.set_velocity(linear, angular)

# Create subscriber
cmd_sub = bridge.create_subscription(
    Twist,
    "/cmd_vel",
    cmd_vel_callback
)
```

## Robot Control

### Joint Control via ROS2

```python
from std_msgs.msg import Float64MultiArray

def joint_command_callback(msg):
    positions = msg.data
    robot.set_joint_positions(positions)

joint_sub = bridge.create_subscription(
    Float64MultiArray,
    "/joint_commands",
    joint_command_callback
)
```

## TF Publishing

### Publishing Transform Tree

```python
from geometry_msgs.msg import TransformStamped
import tf2_ros

# Create TF broadcaster
tf_broadcaster = tf2_ros.TransformBroadcaster()

# Publish transform
transform = TransformStamped()
transform.header.frame_id = "base_link"
transform.child_frame_id = "camera_link"
transform.transform.translation.x = 0.1
transform.transform.translation.y = 0.0
transform.transform.translation.z = 0.2
tf_broadcaster.sendTransform(transform)
```

## Services

### Providing Services

```python
from std_srvs.srv import SetBool

def reset_service_callback(request, response):
    # Reset simulation
    simulation_context.reset()
    response.success = True
    return response

# Create service
reset_service = bridge.create_service(
    SetBool,
    "/reset_simulation",
    reset_service_callback
)
```

## Actions

### Action Server

```python
from example_interfaces.action import Fibonacci

def execute_fibonacci(goal_handle):
    # Execute action
    result = Fibonacci.Result()
    result.sequence = [0, 1, 1, 2, 3, 5]
    goal_handle.succeed()
    return result

# Create action server
action_server = bridge.create_action_server(
    Fibonacci,
    "/fibonacci",
    execute_fibonacci
)
```

## Complete Example

```python
from omni.isaac.kit import SimulationApp
from omni.isaac.core import SimulationContext
from omni.isaac.ros2_bridge import ROS2Bridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

simulation_app = SimulationApp({"headless": False})

# Initialize
simulation_context = SimulationContext()
bridge = ROS2Bridge()
bridge.initialize()

# Create publishers and subscribers
joint_pub = bridge.create_publisher(JointState, "/joint_states")
cmd_sub = bridge.create_subscription(Twist, "/cmd_vel", cmd_vel_callback)

# Simulation loop
while simulation_app.is_running():
    simulation_context.step()
    # Publish joint states
    publish_joint_states()
    rclpy.spin_once(bridge, timeout_sec=0.0)

simulation_app.close()
```

## Best Practices

- Use appropriate message types
- Set correct frame IDs
- Handle timing correctly
- Test with real ROS2 nodes
- Monitor topic bandwidth

## Troubleshooting

- **No messages received**: Check topic names and types
- **Timing issues**: Adjust simulation timestep
- **TF errors**: Verify frame hierarchy
- **Performance**: Optimize publish rates

## Module Summary

Congratulations! You've completed Module 3. You now understand:
- Isaac Sim architecture and features
- Scene composition and asset management
- Physics and rendering configuration
- ROS2 integration

## Next Module

In Module 4, we'll explore Vision-Language-Action (VLA) models for embodied AI.

