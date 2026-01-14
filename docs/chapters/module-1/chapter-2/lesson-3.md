---
sidebar_position: 4
title: 'Lesson 2.3: Robot Control with rclpy'
---

# Lesson 2.3: Robot Control with rclpy

## Learning Objectives

After completing this lesson, you will be able to:
- Implement robot control algorithms using rclpy
- Create control nodes that interact with robot hardware
- Handle sensor feedback and actuator commands

## Key Concepts

### Robot Control in ROS 2

Robot control in ROS 2 typically involves:
- Reading sensor data from topics
- Computing control commands
- Publishing commands to actuator topics
- Using feedback to adjust control behavior

### Control Loop Architecture

A typical robot control node follows this pattern:
- Subscribe to sensor topics
- Implement control algorithm
- Publish commands to actuator topics
- Optionally use services for high-level commands

## Basic Control Node Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Create subscriber for joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        # Create publisher for joint commands
        self.joint_cmd_pub = self.create_publisher(
            Float64,
            '/position_controller/commands',
            10)

        # Create timer for control loop
        self.control_timer = self.create_timer(0.01, self.control_loop)

        self.current_positions = {}
        self.target_positions = {}

    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            self.current_positions[name] = msg.position[i]

    def control_loop(self):
        # Implement your control algorithm here
        cmd_msg = Float64()
        cmd_msg.data = 0.0  # Placeholder - implement actual control
        self.joint_cmd_pub.publish(cmd_msg)
```

## Exercise

Create a simple PID controller node that controls a joint position based on sensor feedback.

## Summary

In this lesson, you learned how to implement robot control systems using rclpy, including subscribing to sensor data and publishing control commands.