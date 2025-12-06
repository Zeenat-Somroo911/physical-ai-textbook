---
sidebar_position: 2
---

# Project 1: Multi-Robot Communication System

## Project Overview

Build a multi-robot communication system where multiple robots can coordinate and share information using ROS 2. This project demonstrates fundamental ROS 2 concepts including nodes, topics, services, and namespaces.

### Learning Objectives

- Create and manage multiple ROS 2 nodes
- Implement publisher-subscriber communication
- Use ROS 2 namespaces for multi-robot systems
- Implement service-based coordination
- Handle multiple robots in a single workspace

### Prerequisites

- Completed [Module 1: ROS 2 Fundamentals](../module-01-ros2/introduction)
- Basic Python programming
- ROS 2 Humble installed
- Understanding of topics and services

## Project Requirements

### Functional Requirements

1. **Robot Nodes**: Create at least 2 robot nodes that can move and communicate
2. **Position Sharing**: Robots publish their position to a shared topic
3. **Coordination Service**: Implement a service for robots to request coordination
4. **Collision Avoidance**: Robots avoid each other based on shared position data
5. **Status Monitoring**: Monitor all robots' status in real-time

### Technical Requirements

- Use ROS 2 Python (`rclpy`)
- Implement proper namespacing (`robot1/`, `robot2/`)
- Use topics for position sharing
- Use services for coordination
- Include error handling and logging

## Step-by-Step Implementation

### Step 1: Create ROS 2 Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python multi_robot_system \
    --dependencies rclpy std_msgs geometry_msgs example_interfaces
cd ~/ros2_ws
colcon build --packages-select multi_robot_system
source install/setup.bash
```

### Step 2: Robot Node Implementation

Create `multi_robot_system/multi_robot_system/robot_node.py`:

```python
#!/usr/bin/env python3
"""
Multi-Robot Node

Each robot publishes its position and subscribes to other robots' positions.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts
import random
import math

class RobotNode(Node):
    def __init__(self, robot_name):
        super().__init__(f'{robot_name}_node')
        
        self.robot_name = robot_name
        self.position = Point()
        self.position.x = random.uniform(-5.0, 5.0)
        self.position.y = random.uniform(-5.0, 5.0)
        self.position.z = 0.0
        
        # Publishers
        self.position_pub = self.create_publisher(
            Point,
            f'/{robot_name}/position',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            f'/{robot_name}/status',
            10
        )
        
        # Subscribers (subscribe to all robots' positions)
        self.other_robots = {}
        self.create_subscription(
            Point,
            '/robot1/position',
            lambda msg: self.position_callback('robot1', msg),
            10
        )
        self.create_subscription(
            Point,
            '/robot2/position',
            lambda msg: self.position_callback('robot2', msg),
            10
        )
        
        # Service for coordination
        self.coordination_srv = self.create_service(
            AddTwoInts,
            f'/{robot_name}/coordinate',
            self.coordination_callback
        )
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            f'/{robot_name}/cmd_vel',
            10
        )
        
        # Timer for periodic updates
        self.timer = self.create_timer(0.1, self.update_callback)
        
        self.get_logger().info(f'{robot_name} node started at ({self.position.x:.2f}, {self.position.y:.2f})')
    
    def position_callback(self, robot_name, msg):
        """Handle position updates from other robots."""
        if robot_name != self.robot_name:
            self.other_robots[robot_name] = msg
            distance = math.sqrt(
                (msg.x - self.position.x)**2 + 
                (msg.y - self.position.y)**2
            )
            if distance < 1.0:
                self.get_logger().warn(
                    f'Warning: {robot_name} is close! Distance: {distance:.2f}m'
                )
    
    def coordination_callback(self, request, response):
        """Handle coordination service requests."""
        self.get_logger().info(
            f'Coordination request: a={request.a}, b={request.b}'
        )
        response.sum = request.a + request.b
        return response
    
    def update_callback(self):
        """Periodic update: publish position and check for collisions."""
        # Publish current position
        self.position_pub.publish(self.position)
        
        # Publish status
        status_msg = String()
        status_msg.data = f'{self.robot_name}: OK at ({self.position.x:.2f}, {self.position.y:.2f})'
        self.status_pub.publish(status_msg)
        
        # Simple collision avoidance
        cmd = Twist()
        min_distance = float('inf')
        closest_robot = None
        
        for name, pos in self.other_robots.items():
            distance = math.sqrt(
                (pos.x - self.position.x)**2 + 
                (pos.y - self.position.y)**2
            )
            if distance < min_distance:
                min_distance = distance
                closest_robot = pos
        
        # Avoid collision
        if min_distance < 1.5 and closest_robot:
            # Move away from closest robot
            dx = self.position.x - closest_robot.x
            dy = self.position.y - closest_robot.y
            angle = math.atan2(dy, dx)
            
            cmd.linear.x = 0.3
            cmd.angular.z = angle * 0.5
        else:
            # Random movement
            cmd.linear.x = 0.2
            cmd.angular.z = random.uniform(-0.5, 0.5)
        
        self.cmd_vel_pub.publish(cmd)
        
        # Update position (simplified - in real robot, use odometry)
        self.position.x += cmd.linear.x * 0.1 * math.cos(cmd.angular.z)
        self.position.y += cmd.linear.x * 0.1 * math.sin(cmd.angular.z)

def main(args=None):
    rclpy.init(args=args)
    
    # Get robot name from command line argument
    import sys
    robot_name = sys.argv[1] if len(sys.argv) > 1 else 'robot1'
    
    node = RobotNode(robot_name)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f'{robot_name} shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Monitor Node

Create `multi_robot_system/multi_robot_system/monitor_node.py`:

```python
#!/usr/bin/env python3
"""
Monitor Node

Monitors all robots' status and positions.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point

class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')
        
        # Subscribe to all robot statuses
        self.robot_statuses = {}
        
        self.create_subscription(
            String,
            '/robot1/status',
            lambda msg: self.status_callback('robot1', msg),
            10
        )
        
        self.create_subscription(
            String,
            '/robot2/status',
            lambda msg: self.status_callback('robot2', msg),
            10
        )
        
        # Subscribe to positions
        self.robot_positions = {}
        
        self.create_subscription(
            Point,
            '/robot1/position',
            lambda msg: self.position_callback('robot1', msg),
            10
        )
        
        self.create_subscription(
            Point,
            '/robot2/position',
            lambda msg: self.position_callback('robot2', msg),
            10
        )
        
        # Timer for periodic status display
        self.timer = self.create_timer(1.0, self.display_status)
        
        self.get_logger().info('Monitor node started')
    
    def status_callback(self, robot_name, msg):
        """Handle status updates."""
        self.robot_statuses[robot_name] = msg.data
    
    def position_callback(self, robot_name, msg):
        """Handle position updates."""
        self.robot_positions[robot_name] = msg
    
    def display_status(self):
        """Display all robots' status."""
        self.get_logger().info('=' * 50)
        self.get_logger().info('ROBOT STATUS MONITOR')
        self.get_logger().info('=' * 50)
        
        for robot_name in ['robot1', 'robot2']:
            status = self.robot_statuses.get(robot_name, 'Unknown')
            pos = self.robot_positions.get(robot_name)
            
            if pos:
                self.get_logger().info(
                    f'{robot_name}: {status} | '
                    f'Position: ({pos.x:.2f}, {pos.y:.2f})'
                )
            else:
                self.get_logger().info(f'{robot_name}: {status} | Position: Unknown')
        
        self.get_logger().info('=' * 50)

def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Monitor shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 4: Coordinator Node

Create `multi_robot_system/multi_robot_system/coordinator_node.py`:

```python
#!/usr/bin/env python3
"""
Coordinator Node

Coordinates multiple robots using services.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from std_msgs.msg import String

class CoordinatorNode(Node):
    def __init__(self):
        super().__init__('coordinator_node')
        
        # Service clients for each robot
        self.robot1_client = self.create_client(AddTwoInts, '/robot1/coordinate')
        self.robot2_client = self.create_client(AddTwoInts, '/robot2/coordinate')
        
        # Wait for services
        self.get_logger().info('Waiting for robot services...')
        self.robot1_client.wait_for_service()
        self.robot2_client.wait_for_service()
        self.get_logger().info('All robot services available!')
        
        # Timer for periodic coordination
        self.timer = self.create_timer(5.0, self.coordinate_robots)
        
        self.get_logger().info('Coordinator node started')
    
    def coordinate_robots(self):
        """Coordinate robots using services."""
        self.get_logger().info('Coordinating robots...')
        
        # Send coordination request to robot1
        request1 = AddTwoInts.Request()
        request1.a = 10
        request1.b = 20
        
        future1 = self.robot1_client.call_async(request1)
        future1.add_done_callback(
            lambda future: self.get_logger().info(
                f'Robot1 response: {future.result().sum}'
            )
        )
        
        # Send coordination request to robot2
        request2 = AddTwoInts.Request()
        request2.a = 30
        request2.b = 40
        
        future2 = self.robot2_client.call_async(request2)
        future2.add_done_callback(
            lambda future: self.get_logger().info(
                f'Robot2 response: {future.result().sum}'
            )
        )

def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Coordinator shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 5: Update setup.py

Update `multi_robot_system/setup.py`:

```python
from setuptools import setup

setup(
    name='multi_robot_system',
    version='0.0.1',
    packages=['multi_robot_system'],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'robot_node = multi_robot_system.robot_node:main',
            'monitor_node = multi_robot_system.monitor_node:main',
            'coordinator_node = multi_robot_system.coordinator_node:main',
        ],
    },
)
```

### Step 6: Launch File

Create `multi_robot_system/launch/multi_robot.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Robot 1
        Node(
            package='multi_robot_system',
            executable='robot_node',
            name='robot1_node',
            arguments=['robot1'],
            output='screen'
        ),
        
        # Robot 2
        Node(
            package='multi_robot_system',
            executable='robot_node',
            name='robot2_node',
            arguments=['robot2'],
            output='screen'
        ),
        
        # Monitor
        Node(
            package='multi_robot_system',
            executable='monitor_node',
            name='monitor_node',
            output='screen'
        ),
        
        # Coordinator
        Node(
            package='multi_robot_system',
            executable='coordinator_node',
            name='coordinator_node',
            output='screen'
        ),
    ])
```

## Testing Steps

### Test 1: Basic Node Communication

```bash
# Terminal 1: Start robot1
ros2 run multi_robot_system robot_node robot1

# Terminal 2: Start robot2
ros2 run multi_robot_system robot_node robot2

# Terminal 3: Monitor topics
ros2 topic echo /robot1/position
ros2 topic echo /robot2/position
```

**Expected Output**: Both robots should publish position updates.

### Test 2: Service Communication

```bash
# Terminal 1: Start coordinator
ros2 run multi_robot_system coordinator_node

# Terminal 2: Check services
ros2 service list
ros2 service call /robot1/coordinate example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
```

**Expected Output**: Service should return sum = 8.

### Test 3: Complete System

```bash
# Launch all nodes
ros2 launch multi_robot_system multi_robot.launch.py

# In another terminal, monitor
ros2 topic echo /robot1/status
ros2 topic echo /robot2/status
```

**Expected Output**: 
- Both robots publishing positions
- Monitor displaying status
- Coordinator sending requests
- Collision avoidance working

### Test 4: Collision Avoidance

```bash
# Start system
ros2 launch multi_robot_system multi_robot.launch.py

# Watch for collision warnings
# Robots should move away when within 1.5m
```

**Expected Output**: Warning messages when robots are close, and robots changing direction.

## Expected Outputs

### Terminal Output Example

```
[robot1_node]: robot1 node started at (2.34, -1.56)
[robot2_node]: robot2 node started at (-3.21, 4.12)
[monitor_node]: Monitor node started
[coordinator_node]: Coordinator node started
[monitor_node]: ==================================================
[monitor_node]: ROBOT STATUS MONITOR
[monitor_node]: ==================================================
[monitor_node]: robot1: robot1: OK at (2.34, -1.56) | Position: (2.34, -1.56)
[monitor_node]: robot2: robot2: OK at (-3.21, 4.12) | Position: (-3.21, 4.12)
[coordinator_node]: Coordinating robots...
[robot1_node]: Coordination request: a=10, b=20
[robot2_node]: Coordination request: a=30, b=40
[coordinator_node]: Robot1 response: 30
[coordinator_node]: Robot2 response: 70
```

## Grading Rubric

### Functionality (40 points)

- **Robot Nodes** (10 points)
  - ✅ Two robot nodes created and running
  - ✅ Proper namespacing implemented
  - ✅ Position publishing working

- **Communication** (10 points)
  - ✅ Topics for position sharing
  - ✅ Services for coordination
  - ✅ All nodes communicating correctly

- **Collision Avoidance** (10 points)
  - ✅ Detects nearby robots
  - ✅ Changes direction when too close
  - ✅ Warning messages displayed

- **Monitoring** (10 points)
  - ✅ Monitor node displays all robot statuses
  - ✅ Real-time position updates
  - ✅ Clear status display

### Code Quality (30 points)

- **Structure** (10 points): Well-organized, modular code
- **Documentation** (10 points): Comments and docstrings
- **Error Handling** (10 points): Proper exception handling

### ROS 2 Best Practices (20 points)

- **Namespacing** (5 points): Proper use of namespaces
- **QoS** (5 points): Appropriate QoS settings
- **Logging** (5 points): Appropriate log levels
- **Launch Files** (5 points): Proper launch file structure

### Testing (10 points)

- **Test Coverage** (5 points): All components tested
- **Documentation** (5 points): Test results documented

## Extensions

### Extension 1: Add More Robots

- Support 3+ robots
- Dynamic robot discovery
- Scalable architecture

### Extension 2: Advanced Coordination

- Task assignment
- Formation control
- Swarm behaviors

### Extension 3: Visualization

- RViz2 visualization
- Real-time robot positions
- Communication graph

## Troubleshooting

### Issue: Robots not communicating

**Solution**: Check topic names match, verify namespaces are correct.

### Issue: Services not found

**Solution**: Ensure services are created before clients try to connect.

### Issue: Collision avoidance not working

**Solution**: Verify position callbacks are receiving messages, check distance calculation.

## Next Steps

- [Project 2: Build Your Own Humanoid](project-02-simulation.md) - Simulation project
- [Module 2: Simulation & Gazebo](../module-02-simulation/gazebo-intro) - Learn more about simulation

---

**Congratulations on completing Project 1!** You've built a multi-robot communication system using ROS 2. 🤖

