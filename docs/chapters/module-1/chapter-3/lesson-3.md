---
sidebar_position: 4
title: 'Lesson 3.3: Humanoid Robot Models'
---

# Lesson 3.3: Humanoid Robot Models

## Learning Objectives

After completing this lesson, you will be able to:
- Create complex humanoid robot models using URDF
- Implement proper kinematic chains for bipedal locomotion
- Model anthropomorphic features in URDF

## Key Concepts

### Humanoid Robot Structure

Humanoid robots typically have:
- Torso/trunk as the central body
- Head with sensors
- Two arms with manipulators
- Two legs for locomotion
- Appropriate degrees of freedom for human-like movement

### Joint Limitations

Humanoid models require careful consideration of joint limits to ensure realistic movement.

### Denavit-Hartenberg Parameters

Understanding DH parameters helps in designing proper kinematic chains for limbs.

## Humanoid URDF Example

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
  </link>

  <!-- Left Arm -->
  <joint name="left_shoulder_pitch" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0.1 0.1" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </visual>
  </link>

  <!-- Similar joints for elbow, wrist, etc. -->
</robot>
```

## Exercise

Create a simplified humanoid model with torso, head, and one arm, including proper joint limits.

## Summary

In this lesson, you learned about creating complex humanoid robot models in URDF, including proper kinematic chains and anthropomorphic features.