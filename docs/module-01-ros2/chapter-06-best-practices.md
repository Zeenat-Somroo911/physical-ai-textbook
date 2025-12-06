---
sidebar_position: 7
---

# Chapter 6: ROS2 Best Practices

## Introduction

This chapter covers best practices for developing robust, maintainable, and efficient ROS2 applications. Learn from common patterns and avoid common pitfalls.

## Code Organization

### Package Structure

```
my_robot_package/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── setup.cfg
├── my_robot_package/
│   ├── __init__.py
│   ├── nodes/
│   │   ├── __init__.py
│   │   └── my_node.py
│   ├── modules/
│   │   ├── __init__.py
│   │   └── utilities.py
│   └── msg/
│       └── CustomMessage.msg
├── launch/
│   └── my_launch.py
├── config/
│   └── params.yaml
└── test/
    └── test_my_node.py
```

### Naming Conventions

- **Packages**: `snake_case` (e.g., `my_robot_package`)
- **Nodes**: Descriptive names (e.g., `sensor_driver_node`)
- **Topics**: `snake_case` with namespace (e.g., `/robot/sensor_data`)
- **Services**: `snake_case` (e.g., `/robot/get_status`)

## Error Handling

```python
import rclpy
from rclpy.node import Node

class RobustNode(Node):
    def __init__(self):
        super().__init__('robust_node')
        try:
            self.publisher_ = self.create_publisher(String, 'topic', 10)
        except Exception as e:
            self.get_logger().error(f'Failed to create publisher: {e}')
            raise
```

## Logging

```python
# Use appropriate log levels
self.get_logger().debug('Detailed debugging information')
self.get_logger().info('General information')
self.get_logger().warn('Warning message')
self.get_logger().error('Error occurred')
self.get_logger().fatal('Fatal error')
```

## Resource Management

### Timers

```python
# Clean up timers properly
def destroy_node(self):
    self.timer.destroy()
    super().destroy_node()
```

### Subscriptions

```python
# Use appropriate queue sizes
self.subscription = self.create_subscription(
    String,
    'topic',
    self.callback,
    10  # Queue size - adjust based on message rate
)
```

## Performance Optimization

### Avoid Blocking in Callbacks

```python
# Bad: Blocking operation in callback
def callback(self, msg):
    time.sleep(1.0)  # Blocks the executor
    self.process(msg)

# Good: Use async or separate thread
def callback(self, msg):
    # Queue for processing
    self.message_queue.put(msg)
```

### Efficient Message Handling

```python
# Reuse message objects when possible
self.msg = String()

def timer_callback(self):
    self.msg.data = f'Count: {self.i}'
    self.publisher_.publish(self.msg)
    self.i += 1
```

## Testing

### Unit Tests

```python
import unittest
import rclpy
from my_package.nodes.my_node import MyNode

class TestMyNode(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = MyNode()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_initialization(self):
        self.assertIsNotNone(self.node)
```

## Documentation

### Code Comments

```python
class MyNode(Node):
    """
    A node that processes sensor data.
    
    This node subscribes to sensor topics and processes
    the incoming data to extract meaningful information.
    """
    def __init__(self):
        super().__init__('my_node')
```

### README Files

Always include:
- Package description
- Installation instructions
- Usage examples
- Parameter documentation
- Launch file descriptions

## Security

- Use ROS2 security features in production
- Validate all inputs
- Use namespaces to isolate components
- Set appropriate permissions

## Version Control

- Use `.gitignore` for build artifacts
- Commit `package.xml` and `CMakeLists.txt`
- Document dependencies
- Tag releases

## Common Pitfalls

1. **Forgetting to spin**: Always call `rclpy.spin()` or use an executor
2. **Wrong message types**: Ensure publisher and subscriber use the same type
3. **Queue overflow**: Set appropriate queue sizes
4. **Namespace issues**: Be aware of node namespaces
5. **Parameter not declared**: Always declare parameters before use

## Exercises

1. Refactor a simple node following best practices
2. Add comprehensive error handling
3. Write unit tests for your nodes
4. Create proper documentation

## Module Summary

Congratulations! You've completed Module 1. You now understand:
- ROS2 fundamentals
- Nodes, topics, services, and actions
- Parameters and launch files
- Best practices for ROS2 development

## Next Module

In Module 2, we'll explore simulation and Gazebo, learning how to create and simulate robots in virtual environments.

