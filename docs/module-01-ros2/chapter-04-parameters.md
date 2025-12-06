---
sidebar_position: 5
---

# Chapter 4: Parameters

## Introduction

Parameters allow you to configure nodes at runtime without modifying code. This chapter covers parameter declaration, access, and management in ROS2.

## What are Parameters?

Parameters are key-value pairs that can be:
- Set at node startup
- Modified at runtime
- Queried by other nodes
- Saved and loaded from files

## Declaring Parameters

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with default values
        self.declare_parameter('my_string', 'world')
        self.declare_parameter('my_int', 42)
        self.declare_parameter('my_double', 3.14)
        self.declare_parameter('my_bool', True)
        
        # Get parameter values
        my_string = self.get_parameter('my_string').get_parameter_value().string_value
        my_int = self.get_parameter('my_int').get_parameter_value().integer_value
        
        self.get_logger().info(f'String parameter: {my_string}')
        self.get_logger().info(f'Integer parameter: {my_int}')
```

## Setting Parameters

### From Command Line

```bash
ros2 run my_package my_node --ros-args \
  -p my_string:=hello \
  -p my_int:=100
```

### From Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            parameters=[{
                'my_string': 'hello',
                'my_int': 100,
                'my_double': 2.71
            }]
        )
    ])
```

### From YAML File

```yaml
my_node:
  ros__parameters:
    my_string: "hello"
    my_int: 100
    my_double: 2.71
```

```python
Node(
    package='my_package',
    executable='my_node',
    parameters=['path/to/config.yaml']
)
```

## Parameter Types

ROS2 supports these parameter types:
- `bool`
- `int`
- `double`
- `string`
- `byte_array`
- `bool_array`
- `int_array`
- `double_array`
- `string_array`

## Parameter Descriptors

You can provide additional information about parameters:

```python
from rclpy.parameter import ParameterDescriptor

descriptor = ParameterDescriptor(
    description='A string parameter',
    type=ParameterType.PARAMETER_STRING,
    read_only=False
)
self.declare_parameter('my_string', 'default', descriptor)
```

## Parameter Constraints

```python
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange

descriptor = ParameterDescriptor(
    description='An integer between 0 and 100',
    integer_range=[IntegerRange(from_value=0, to_value=100, step=1)]
)
self.declare_parameter('my_int', 50, descriptor)
```

## Querying Parameters

```bash
# List all parameters
ros2 param list

# Get parameter value
ros2 param get /my_node my_string

# Set parameter at runtime
ros2 param set /my_node my_int 200
```

## Best Practices

- Use descriptive parameter names
- Provide meaningful default values
- Document parameters with descriptors
- Validate parameter values
- Use parameter files for complex configurations

## Exercises

1. Create a node with multiple parameters
2. Load parameters from a YAML file
3. Implement parameter validation

## Next Steps

Chapter 5 will cover launch files, which help orchestrate multiple nodes and configurations.

