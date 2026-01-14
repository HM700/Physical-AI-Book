---
sidebar_position: 2
title: 'Lesson 3.1: Introduction to URDF'
---

# Lesson 3.1: Introduction to URDF

## Learning Objectives

After completing this lesson, you will be able to:
- Understand what URDF (Unified Robot Description Format) is and its purpose
- Create basic URDF files to describe robot geometry
- Visualize simple robot models in RViz

## Key Concepts

### What is URDF?

URDF (Unified Robot Description Format) is an XML format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including:
- Links (rigid parts of the robot)
- Joints (connections between links)
- Materials and visual properties

### URDF Structure

A basic URDF consists of:
- Links: Represent rigid bodies of the robot
- Joints: Define how links connect and move relative to each other
- Visual: Defines how the robot appears in visualization
- Collision: Defines collision properties for physics simulation

## Basic URDF Example

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Child link connected by joint -->
  <link name="child_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.2"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>

  <!-- Joint connecting base and child -->
  <joint name="base_to_child" type="fixed">
    <parent link="base_link"/>
    <child link="child_link"/>
    <origin xyz="0.3 0 0.2" rpy="0 0 0"/>
  </joint>
</robot>
```

## Exercise

Create a simple URDF file for a robot with a rectangular base and cylindrical head.

## Summary

In this lesson, you learned about URDF, the Unified Robot Description Format used in ROS to describe robot models, and created a basic robot model.