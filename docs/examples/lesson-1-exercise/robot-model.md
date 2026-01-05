---
title: Basic Robot Model Setup
---

# Basic Robot Model Setup

This exercise provides the files and instructions for setting up a basic humanoid robot model in simulation.

## Files Included

### URDF Model (Robot Definition)
```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="head_joint" type="fixed">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.2"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.0025" ixy="0" ixz="0" iyy="0.0025" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <joint name="left_shoulder" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

## Setup Instructions

1. Save the URDF code above as `simple_humanoid.urdf`
2. Load the model into your simulation environment
3. Verify that all links and joints are properly loaded
4. Test basic joint movement

## Exercise Steps

1. **Load the model**: Load the URDF file into your simulation environment
2. **Verify joints**: Check that the head joint is fixed and the shoulder joint is revolute
3. **Test movement**: Apply a command to move the shoulder joint and observe the response
4. **Record observations**: Document how the robot responds to commands

## Expected Results

- Robot model loads successfully in simulation
- Head remains fixed relative to the base
- Left arm moves when joint commands are applied
- No errors or warnings during loading

## Troubleshooting

If the model doesn't load properly:
- Check that all links have valid geometry definitions
- Verify joint connections are properly specified
- Ensure all required dependencies are installed