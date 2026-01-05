---
title: Quick Reference Guide
---

# Quick Reference Guide

This guide provides quick access to common concepts, formulas, and procedures used in humanoid robotics.

## Common Formulas

### Forward Kinematics (2D Planar Arm)
For a 2D arm with two links of length L1 and L2:
```
x = L1 * cos(θ1) + L2 * cos(θ1 + θ2)
y = L1 * sin(θ1) + L2 * sin(θ1 + θ2)
```

### Inverse Kinematics (2D Planar Arm)
For a 2D arm with end-effector position (x, y):
```
r² = x² + y²
θ2 = ±arccos((L1² + L2² - r²) / (2 * L1 * L2))
θ1 = arctan2(y, x) - arctan2(L2 * sin(θ2), L1 + L2 * cos(θ2))
```

### Rotation Matrix (2D)
For rotation by angle θ:
```
R = [cos(θ)  -sin(θ)]
    [sin(θ)   cos(θ)]
```

## Sensor Specifications

| Sensor Type | Typical Use | Advantages | Limitations |
|-------------|-------------|------------|-------------|
| Camera | Vision, object recognition | Rich visual data | Lighting dependent, limited depth |
| LIDAR | Mapping, obstacle detection | Accurate distance, 3D data | Expensive, weather sensitive |
| IMU | Orientation, motion | Fast response, compact | Drifts over time |
| Force/Torque | Contact sensing | Direct force measurement | Requires physical contact |

## Common ROS Commands

```bash
# Launch a package
ros2 launch package_name launch_file.py

# List active topics
ros2 topic list

# Echo a topic
ros2 topic echo /topic_name std_msgs/msg/String

# Run a node
ros2 run package_name node_name

# List active services
ros2 service list
```

## Simulation Platforms Comparison

| Platform | Best For | Key Features | Learning Curve |
|----------|----------|--------------|----------------|
| ROS 2 | Real robotics, research | Large community, many packages | High |
| Gazebo | Physics simulation | Realistic physics, sensor simulation | Medium |
| Unity | Visual quality, gaming | High-quality graphics, VR/AR | Medium |
| NVIDIA Isaac | AI/ML, perception | GPU acceleration, AI tools | High |

## Troubleshooting

### Robot Not Moving
1. Check if all joints are properly loaded
2. Verify actuator commands are being sent
3. Confirm no safety limits are being triggered

### Perception Errors
1. Verify sensor calibration
2. Check lighting conditions (for cameras)
3. Ensure no sensor interference

### Simulation Issues
1. Verify simulation environment is properly loaded
2. Check for missing dependencies
3. Confirm model files are accessible

## Safety Guidelines

1. Always test in simulation before real hardware
2. Implement safety limits on all actuators
3. Use emergency stop mechanisms
4. Ensure proper supervision during testing
5. Follow all manufacturer guidelines for hardware