---
sidebar_position: 3
title: 'Lesson 1.2: ROS 2 Topics and Message Passing'
---

# Lesson 1.2: ROS 2 Topics and Message Passing

## Learning Objectives

After completing this lesson, you will be able to:
- Understand the concept of topics and message passing in ROS 2
- Create publishers and subscribers
- Implement basic communication between nodes

## Key Concepts

### Topics in ROS 2

Topics are named buses over which nodes exchange messages. A node can publish messages to a topic and/or subscribe to a topic to receive messages.

### Publisher-Subscriber Pattern

- Publishers send messages to a topic
- Subscribers receive messages from a topic
- Multiple publishers and subscribers can use the same topic

## Publisher Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
```

## Subscriber Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
```

## Exercise

Create a publisher-subscriber pair that exchanges custom messages containing robot sensor data.

## Summary

In this lesson, you learned about topics and message passing in ROS 2, which form the backbone of communication between nodes in a ROS 2 system.