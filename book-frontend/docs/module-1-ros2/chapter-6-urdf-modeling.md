---
sidebar_label: 'Chapter 6: URDF Modeling'
sidebar_position: 6
title: 'Chapter 6: Humanoid Robot Modeling with URDF'
description: 'Creating robot models using Unified Robot Description Format for simulation and control'
---

# Chapter 6: Humanoid Robot Modeling with URDF

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including links (rigid parts), joints (connections between parts), and their spatial relationships.

## Links and Joints

A robot model in URDF consists of:

- **Links**: Rigid bodies with mass, visual representation, and collision properties
- **Joints**: Connections between links that define how they can move relative to each other

Each robot has a single base link (root) with all other links connected in a tree structure.

## Link Definition

Links contain information about:
- Visual appearance (meshes, colors, materials)
- Collision geometry (for physics simulation)
- Inertial properties (mass, center of mass, moments of inertia)

```xml
<link name="link_name">
  <visual>
    <geometry>
      <cylinder length="0.1" radius="0.05"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 0.8 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.1" radius="0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
  </inertial>
</link>
```

## Joint Types

URDF supports several joint types:
- **Fixed**: No movement between links
- **Revolute**: Rotational movement around an axis (with limits)
- **Continuous**: Unlimited rotational movement
- **Prismatic**: Linear sliding movement (with limits)
- **Floating**: 6 degrees of freedom
- **Planar**: Movement in a plane

```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>
```

## URDF for Humanoid Robots

Humanoid robot models have specific characteristics:
- Bipedal structure with legs, torso, arms, and head
- Multiple degrees of freedom for walking and manipulation
- Complex kinematic chains for arms and legs
- Specialized joints for realistic movement

## Kinematic Chains and Forward Kinematics

URDF models enable:
- Forward kinematics: calculating end-effector position from joint angles
- Inverse kinematics: calculating joint angles for desired end-effector position
- Dynamics simulation: calculating forces and torques

## Integration with ROS 2

URDF integrates with ROS 2 through:
- `robot_state_publisher`: publishes TF transforms for visualization
- `joint_state_publisher`: publishes joint state information
- Gazebo simulation: imports URDF for physics simulation
- MoveIt!: uses URDF for motion planning

## Working with URDF in ROS 2

To load and use a URDF model:

1. Define the robot model in a .urdf or .xacro file
2. Launch the robot_state_publisher node
3. Subscribe to /tf or /tf_static topics for transforms
4. Use the model with simulation or visualization tools

## Xacro: XML Macros for URDF

Xacro extends URDF with macros, variables, and expressions to simplify complex robot models:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:macro name="wheel" params="prefix">
    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <sphere radius="${wheel_radius}"/>
        </geometry>
      </visual>
    </link>
  </xacro:macro>
</robot>
```

## Visualization and Debugging

URDF models can be visualized and debugged using:
- RViz: 3D visualization of robot models
- check_urdf: command-line tool to validate URDF
- urdf_to_graphiz: generates kinematic chain diagrams

## Best Practices

- Organize links and joints hierarchically
- Use consistent naming conventions
- Include proper inertial properties for simulation
- Validate URDF before simulation
- Use Xacro for complex models to avoid repetition

## Summary

URDF provides the foundation for representing humanoid robots in ROS 2, enabling simulation, visualization, and control. Properly modeled URDF files are essential for connecting AI agents to realistic robot simulations and real hardware. The next chapter will tie together all the concepts covered in this module.