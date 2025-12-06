---
sidebar_position: 2
---

# Chapter 1: Getting Started with ROS2

## Introduction

ROS2 (Robot Operating System 2) is the next generation of ROS, designed to address the limitations of ROS1 while maintaining backward compatibility where possible. This chapter will guide you through installation, basic concepts, and your first ROS2 program.

## What is ROS2?

ROS2 is a set of software libraries and tools for building robot applications. It provides:

- **Distributed architecture**: Nodes can run on different machines
- **Real-time capabilities**: Better support for real-time systems
- **Multiple DDS implementations**: Flexible middleware options
- **Improved security**: Built-in security features
- **Cross-platform support**: Works on Linux, Windows, and macOS

## Installation

### Ubuntu/Debian

```bash
sudo apt update
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

### macOS

```bash
brew tap ros/ros2
brew install ros-humble-desktop
```

## Your First ROS2 Program

Let's create a simple ROS2 node in Python:

```python
import rclpy
from rclpy.node import Node

class MyFirstNode(Node):
    def __init__(self):
        super().__init__('my_first_node')
        self.get_logger().info('Hello ROS2!')

def main(args=None):
    rclpy.init(args=args)
    node = MyFirstNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Key Concepts

- **Nodes**: Executable processes that perform computation
- **Topics**: Named buses for exchanging messages
- **Services**: Request-response communication
- **Actions**: Long-running operations with feedback
- **Workspace**: Directory containing your ROS2 packages

## Next Steps

In the next chapter, we'll dive deeper into nodes and topics, the fundamental building blocks of ROS2 communication.

