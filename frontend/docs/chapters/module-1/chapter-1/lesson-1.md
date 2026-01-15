---
sidebar_position: 2
title: 'Lesson 1.1: Understanding ROS 2 Nodes'
---

# Lesson 1.1: Understanding ROS 2 Nodes

## Learning Objectives

After completing this lesson, you will be able to:
- Explain what a ROS 2 node is and its role in the system
- Create a simple ROS 2 node using Python
- Understand node lifecycle and management

## Key Concepts

### What is a ROS 2 Node?

A ROS 2 node is an executable that uses ROS 2 client libraries to communicate with other nodes. Nodes can publish messages to topics, subscribe to topics, provide services, and call services.

### Node Characteristics

- Nodes are the basic execution units of a ROS 2 program
- Multiple nodes can run on the same device or be distributed across multiple devices
- Nodes communicate with each other through topics, services, actions, and parameters

## ROS 2 Node Example

Here's a simple example of a ROS 2 node in Python:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.get_logger().info('Hello from my_node!')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise

Create your first ROS 2 node that prints a custom message to the console.

## Summary

In this lesson, you learned about ROS 2 nodes, which are the fundamental building blocks of any ROS 2 system. Nodes enable communication and coordination between different parts of your robotic system.