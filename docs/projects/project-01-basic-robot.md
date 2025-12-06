---
sidebar_position: 2
---

# Project 1: Basic Robot with ROS2

## Objective

Build a simple mobile robot system using ROS2 that can be controlled via topics and respond to commands.

## Prerequisites

- Completion of Module 1 (ROS2 Fundamentals)
- Basic Python knowledge
- ROS2 Humble installed

## Project Structure

```
basic_robot_package/
├── setup.py
├── setup.cfg
├── package.xml
├── basic_robot_package/
│   ├── __init__.py
│   ├── nodes/
│   │   ├── __init__.py
│   │   ├── robot_controller.py
│   │   └── sensor_publisher.py
│   └── launch/
│       └── robot.launch.py
```

## Step 1: Create ROS2 Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python basic_robot_package
```

## Step 2: Robot Controller Node

Create `basic_robot_package/nodes/robot_controller.py`:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Subscriber for velocity commands
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publisher for robot status
        self.status_pub = self.create_publisher(
            String,
            '/robot_status',
            10
        )
        
        # Timer for status updates
        self.timer = self.create_timer(1.0, self.publish_status)
        
        self.current_velocity = Twist()
        self.get_logger().info('Robot Controller started')
    
    def cmd_vel_callback(self, msg):
        self.current_velocity = msg
        self.get_logger().info(
            f'Received command: linear={msg.linear.x}, angular={msg.angular.z}'
        )
        # Here you would send commands to actual robot hardware
    
    def publish_status(self):
        status_msg = String()
        status_msg.data = f'Velocity: linear={self.current_velocity.linear.x}, angular={self.current_velocity.angular.z}'
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step 3: Sensor Publisher Node

Create `basic_robot_package/nodes/sensor_publisher.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import random

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        
        # Create ultrasonic sensor publisher
        self.ultrasonic_pub = self.create_publisher(
            Range,
            '/ultrasonic_sensor',
            10
        )
        
        # Timer for sensor updates
        self.timer = self.create_timer(0.1, self.publish_sensor_data)
        
        self.get_logger().info('Sensor Publisher started')
    
    def publish_sensor_data(self):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ultrasonic_sensor'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.1
        msg.min_range = 0.02
        msg.max_range = 4.0
        msg.range = random.uniform(0.1, 4.0)  # Simulated distance
        
        self.ultrasonic_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    publisher = SensorPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step 4: Launch File

Create `basic_robot_package/launch/robot.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='basic_robot_package',
            executable='robot_controller',
            name='robot_controller'
        ),
        Node(
            package='basic_robot_package',
            executable='sensor_publisher',
            name='sensor_publisher'
        )
    ])
```

## Step 5: Setup Configuration

Update `setup.py`:

```python
from setuptools import setup

setup(
    name='basic_robot_package',
    version='0.0.1',
    packages=['basic_robot_package'],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'robot_controller = basic_robot_package.nodes.robot_controller:main',
            'sensor_publisher = basic_robot_package.nodes.sensor_publisher:main',
        ],
    },
)
```

## Step 6: Build and Run

```bash
cd ~/ros2_ws
colcon build --packages-select basic_robot_package
source install/setup.bash

# Run launch file
ros2 launch basic_robot_package robot.launch.py

# In another terminal, send commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"

# Monitor topics
ros2 topic echo /robot_status
ros2 topic echo /ultrasonic_sensor
```

## Testing

1. Verify nodes are running: `ros2 node list`
2. Check topics: `ros2 topic list`
3. Test command: Send velocity commands and verify status updates
4. Monitor sensor data: Check ultrasonic sensor readings

## Extensions

- Add obstacle avoidance based on sensor data
- Implement PID controller for precise movement
- Add more sensors (IMU, camera)
- Create a simple navigation stack
- Add service for robot configuration

## Next Project

Once comfortable with this project, move to Project 2: Simulation Setup.

