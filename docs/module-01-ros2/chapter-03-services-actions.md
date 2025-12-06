---
sidebar_position: 4
---

# Chapter 3: Services and Actions

## Introduction

While topics provide one-way communication, services and actions enable two-way, request-response patterns. This chapter covers when and how to use these communication mechanisms.

## Services

**Services** provide synchronous request-response communication. They are ideal for:
- One-time requests
- Operations that return a result immediately
- Configuration changes
- Querying node state

## Creating a Service Server

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}')
        return response
```

## Creating a Service Client

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        self.future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## Actions

**Actions** are designed for long-running tasks that provide feedback. They consist of:
- **Goal**: What you want to achieve
- **Feedback**: Periodic updates on progress
- **Result**: Final outcome

## Creating an Action Server

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci

class ActionServerNode(Node):
    def __init__(self):
        super().__init__('action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result
```

## When to Use What?

- **Topics**: Continuous data streams (sensor data, commands)
- **Services**: Quick request-response (queries, configuration)
- **Actions**: Long-running tasks with feedback (navigation, manipulation)

## Exercises

1. Create a service to control a robot's LED
2. Implement an action for a robot to move to a goal position
3. Compare the performance of services vs actions

## Next Steps

Chapter 4 will cover parameters, which allow runtime configuration of nodes.

