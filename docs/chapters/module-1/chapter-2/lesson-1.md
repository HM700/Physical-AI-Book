---
sidebar_position: 2
title: 'Lesson 2.1: Introduction to rclpy'
---

# Lesson 2.1: Introduction to rclpy

## Learning Objectives

After completing this lesson, you will be able to:
- Understand the rclpy library and its role in ROS 2 Python development
- Set up a basic rclpy node
- Implement basic node functionality with rclpy

## Key Concepts

### What is rclpy?

rclpy is the Python client library for ROS 2. It provides the Python API for developing ROS 2 nodes and communicating with other nodes in the system.

### rclpy Architecture

- rclpy wraps the underlying rcl (ROS Client Library) C API
- Provides Pythonic interfaces to ROS 2 concepts
- Handles memory management and type conversions

## Basic rclpy Node Example

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('Hello from rclpy!')

def main(args=None):
    rclpy.init(args=args)

    my_node = MyNode()

    try:
        rclpy.spin(my_node)
    except KeyboardInterrupt:
        pass
    finally:
        my_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise

Create a basic rclpy node that subscribes to a topic and logs received messages.

## Summary

In this lesson, you learned about rclpy, the Python client library for ROS 2, and how to create basic nodes using it.