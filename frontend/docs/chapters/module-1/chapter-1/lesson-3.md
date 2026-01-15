---
sidebar_position: 4
title: 'Lesson 1.3: ROS 2 Services and Actions'
---

# Lesson 1.3: ROS 2 Services and Actions

## Learning Objectives

After completing this lesson, you will be able to:
- Understand the difference between topics, services, and actions in ROS 2
- Create and use ROS 2 services
- Implement basic action clients and servers

## Key Concepts

### Services in ROS 2

Services provide a request-response communication pattern. A service client sends a request to a service server and waits for a response.

### Actions in ROS 2

Actions are used for long-running tasks that provide feedback during execution. They include goal, result, and feedback mechanisms.

## Service Example

Service definition (add.srv):
```
int64 a
int64 b
---
int64 sum
```

Service Server:
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response
```

Service Client:
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):

    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

    def send_request(self, a, b):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        self.future = self.cli.call_async(request)
        return self.future
```

## Exercise

Create a service that calculates the distance between two coordinate points and returns the result.

## Summary

In this lesson, you learned about services and actions in ROS 2, which provide synchronous and asynchronous request-response communication patterns respectively.