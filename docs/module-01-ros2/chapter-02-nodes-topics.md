---
sidebar_position: 3
---

# Chapter 2: Nodes and Topics

## Introduction

Nodes and topics form the foundation of ROS2's publish-subscribe communication pattern. This chapter explores how nodes communicate through topics.

## Understanding Nodes

A **node** is a process that performs a specific task. Nodes can:
- Publish messages to topics
- Subscribe to topics to receive messages
- Provide services
- Call services from other nodes

## Understanding Topics

A **topic** is a named channel over which nodes exchange messages. Topics use a publish-subscribe pattern:
- **Publisher**: Node that sends messages
- **Subscriber**: Node that receives messages
- **Message**: Data structure sent over the topic

## Creating a Publisher

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1
```

## Creating a Subscriber

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
```

## Running Nodes

```bash
# Terminal 1
ros2 run my_package publisher_node

# Terminal 2
ros2 run my_package subscriber_node
```

## Inspecting Topics

```bash
# List all topics
ros2 topic list

# Echo a topic
ros2 topic echo /chatter

# Get topic info
ros2 topic info /chatter
```

## Custom Messages

You can create custom message types for your specific needs. Messages are defined in `.msg` files.

## Best Practices

- Use descriptive topic names
- Choose appropriate message types
- Set appropriate queue sizes
- Handle message callbacks efficiently

## Exercises

1. Create a publisher that sends sensor data
2. Create a subscriber that processes the sensor data
3. Experiment with different message types

## Next Steps

In Chapter 3, we'll explore services and actions for more structured communication patterns.

