---
sidebar_position: 3
title: 'Lesson 2.2: Advanced rclpy Patterns'
---

# Lesson 2.2: Advanced rclpy Patterns

## Learning Objectives

After completing this lesson, you will be able to:
- Implement complex node architectures using rclpy
- Handle parameters in rclpy nodes
- Use timers and callbacks effectively

## Key Concepts

### Parameters in rclpy

Parameters allow nodes to be configured at runtime. They can be declared and accessed using rclpy's parameter system.

### Timers in rclpy

Timers allow nodes to execute callbacks at regular intervals, which is useful for control loops and periodic tasks.

## Parameter Example

```python
import rclpy
from rclpy.node import Node

class ParamNode(Node):
    def __init__(self):
        super().__init__('param_node')

        # Declare parameters
        self.declare_parameter('frequency', 1.0)
        self.declare_parameter('robot_name', 'my_robot')

        # Get parameter values
        self.frequency = self.get_parameter('frequency').value
        self.robot_name = self.get_parameter('robot_name').value

        self.get_logger().info(f'Frequency: {self.frequency}, Robot: {self.robot_name}')
```

## Timer Example

```python
import rclpy
from rclpy.node import Node

class TimerNode(Node):
    def __init__(self):
        super().__init__('timer_node')

        # Create a timer
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        self.get_logger().info(f'Timer callback #{self.i}')
        self.i += 1
```

## Exercise

Create an rclpy node that uses parameters to control its behavior and a timer to execute periodic tasks.

## Summary

In this lesson, you learned about advanced rclpy patterns including parameters and timers, which are essential for creating configurable and robust ROS 2 nodes.