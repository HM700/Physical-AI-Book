---
title: ROS 2 Kinematics Example
---

# ROS 2 Kinematics Example

This example demonstrates how to implement forward and inverse kinematics in ROS 2.

## Setup

1. Make sure ROS 2 is installed (recommended: Humble Hawksbill)
2. Create a new ROS 2 workspace:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   ```

3. Clone or copy the kinematics example files to your workspace

## Files Included

- `kinematics_node.py` - ROS 2 node implementing kinematics calculations
- `kinematics_msgs.msg` - Custom message definitions for joint angles
- `CMakeLists.txt` - Build configuration
- `package.xml` - Package metadata

## Running the Example

1. Source ROS 2:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select kinematics_examples
   source install/setup.bash
   ```

3. Run the kinematics node:
   ```bash
   ros2 run kinematics_examples kinematics_node
   ```

4. In another terminal, send a target position:
   ```bash
   ros2 topic pub /target_position geometry_msgs/msg/Point "{x: 1.0, y: 0.5, z: 0.0}"
   ```

## Key Concepts Demonstrated

- Using ROS 2 topics for communication
- Implementing kinematics algorithms in a ROS 2 node
- Handling messages for position control
- Service calls for inverse kinematics solutions

## Integration with Other Components

This example can be integrated with:
- Robot simulation environments like Gazebo
- Real robot hardware controllers
- Motion planning algorithms
- Perception systems