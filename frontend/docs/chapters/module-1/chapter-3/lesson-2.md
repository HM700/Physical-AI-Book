---
sidebar_position: 3
title: 'Lesson 3.2: Advanced URDF Features'
---

# Lesson 3.2: Advanced URDF Features

## Learning Objectives

After completing this lesson, you will be able to:
- Use URDF transmissions for controlling joints
- Implement Gazebo-specific extensions in URDF
- Create complex kinematic chains

## Key Concepts

### URDF Transmissions

Transmissions define how actuators connect to joints. They specify the mechanical relationship between actuators and joints.

### Gazebo Extensions

Gazebo-specific tags in URDF allow specifying physics properties and visual effects for simulation.

### Joint Types

URDF supports several joint types:
- revolute: Rotational joint with limited range
- continuous: Rotational joint without limits
- prismatic: Linear sliding joint
- fixed: No movement allowed
- floating: 6DOF movement
- planar: Movement in a plane

## Transmission Example

```xml
<transmission name="simple_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="joint1">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="motor1">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## Gazebo Extension Example

```xml
<gazebo reference="link1">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
</gazebo>

<gazebo>
  <plugin name="control_plugin" filename="libgazebo_ros_control.so">
    <robotNamespace>/my_robot</robotNamespace>
  </plugin>
</gazebo>
```

## Exercise

Extend your URDF model to include a transmission and Gazebo plugin for joint control.

## Summary

In this lesson, you learned about advanced URDF features including transmissions and Gazebo extensions that enable more sophisticated robot models.