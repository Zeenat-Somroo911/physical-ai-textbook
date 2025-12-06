---
sidebar_position: 6
---

# Chapter 5: Launch Files

## Introduction

Launch files allow you to start multiple nodes, set parameters, and configure your ROS2 system with a single command. This chapter covers creating and managing launch files.

## Why Launch Files?

Launch files help you:
- Start multiple nodes simultaneously
- Configure parameters consistently
- Set up complex robot systems
- Organize and document your setup

## Basic Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='my_node',
            output='screen'
        )
    ])
```

## Launching Multiple Nodes

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_package',
            executable='sensor_node',
            name='sensor'
        ),
        Node(
            package='robot_package',
            executable='control_node',
            name='controller'
        ),
        Node(
            package='robot_package',
            executable='actuator_node',
            name='actuator'
        )
    ])
```

## Setting Parameters

```python
Node(
    package='my_package',
    executable='my_node',
    parameters=[{
        'param1': 'value1',
        'param2': 42
    }]
)
```

## Loading Parameter Files

```python
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('my_package'),
        'config',
        'params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            parameters=[config_file]
        )
    ])
```

## Conditional Launching

```python
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim')
    
    return LaunchDescription([
        Node(
            package='robot_package',
            executable='simulation_node',
            condition=IfCondition(use_sim)
        ),
        Node(
            package='robot_package',
            executable='hardware_node',
            condition=IfCondition(['not ', use_sim])
        )
    ])
```

## Including Other Launch Files

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    other_launch = os.path.join(
        get_package_share_directory('other_package'),
        'launch',
        'other.launch.py'
    )
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(other_launch)
        )
    ])
```

## Environment Variables

```python
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),
        Node(
            package='my_package',
            executable='my_node'
        )
    ])
```

## Launch Arguments

```python
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='robot1',
            description='Name of the robot'
        ),
        Node(
            package='robot_package',
            executable='robot_node',
            parameters=[{
                'robot_name': LaunchConfiguration('robot_name')
            }]
        )
    ])
```

## Running Launch Files

```bash
# Basic launch
ros2 launch my_package my_launch.py

# With arguments
ros2 launch my_package my_launch.py robot_name:=robot2 use_sim:=true
```

## Best Practices

- Organize launch files in a `launch/` directory
- Use descriptive names
- Document launch arguments
- Make launch files reusable
- Test launch files thoroughly

## Exercises

1. Create a launch file for a multi-node robot system
2. Add conditional logic based on launch arguments
3. Include other launch files in your main launch file

## Next Steps

Chapter 6 covers best practices for ROS2 development, including code organization, debugging, and optimization.

