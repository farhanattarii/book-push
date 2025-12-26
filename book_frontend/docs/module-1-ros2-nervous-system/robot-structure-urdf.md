---
sidebar_position: 3
title: 'Robot Structure with URDF'
---

# Robot Structure with URDF

## What is URDF?

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. It defines the physical and visual properties of a robot, including:

- Links (rigid bodies)
- Joints (connections between links)
- Visual and collision properties
- Inertial properties
- Sensors and actuators

## URDF for Humanoid Robots

Humanoid robots have complex structures with multiple degrees of freedom. A typical humanoid URDF includes:

- **Trunk**: Torso/chest as the main body
- **Head**: With neck joint for orientation
- **Arms**: Shoulders, elbows, wrists with appropriate joint limits
- **Legs**: Hips, knees, ankles for locomotion
- **End Effectors**: Hands and feet

## Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.2 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.2 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Example joint and link -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>

  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>
</robot>
```

## Joint Types for Humanoid Robots

Humanoid robots require specific joint types for natural movement:

- **Revolute**: Single degree of freedom rotation (like elbow, knee)
- **Continuous**: Like revolute but unlimited rotation (like shoulder)
- **Prismatic**: Linear motion (less common in humanoids)
- **Fixed**: Rigid connection (like attaching sensors)

### Example: Hip Joint Configuration

```xml
<joint name="left_hip_yaw" type="revolute">
  <parent link="torso"/>
  <child link="left_thigh"/>
  <origin xyz="0 -0.15 -0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-0.5" upper="0.5" effort="100" velocity="1.0"/>
  <dynamics damping="1.0" friction="0.1"/>
</joint>
```

## Simulation-Ready URDF Considerations

To ensure your URDF is simulation-ready:

1. **Complete Kinematic Chain**: Ensure all links are connected through joints
2. **Proper Inertial Properties**: Accurate mass and inertia values for realistic physics
3. **Collision Models**: Simplified collision geometry for performance
4. **Gazebo Integration**: Add Gazebo-specific tags if using Gazebo simulator

### Gazebo-Specific Tags

```xml
<gazebo reference="left_foot">
  <material>Gazebo/Blue</material>
  <mu1>0.8</mu1>
  <mu2>0.8</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
</gazebo>
```

## URDF Best Practices for Humanoids

- **Use Standard Names**: Follow ROS conventions for joint names (e.g., `left_hip_yaw`, `right_knee`)
- **Realistic Joint Limits**: Set appropriate limits based on human anatomy
- **Consistent Units**: Use meters for distances, radians for angles
- **Mass Distribution**: Distribute mass realistically across the robot body
- **Origin Placement**: Place origins at joint centers for easier kinematic calculations

## Validation Tools

ROS provides several tools to validate your URDF:

- `check_urdf`: Validates URDF syntax and structure
- `urdf_to_graphiz`: Creates visual representation of the kinematic tree
- RViz: Visualizes the robot model in 3D

## Summary

This chapter covered URDF (Unified Robot Description Format) for humanoid robots, including:

- What URDF is and its key components (links, joints, visual/collision properties)
- How URDF applies specifically to humanoid robot structures
- Basic URDF structure with examples
- Joint types essential for humanoid movement
- Considerations for simulation-ready URDF models
- Best practices for humanoid robotics
- Validation tools for URDF

## Navigation

- **Previous**: [ROS 2 Communication Model](./communication-model.md)
- **Home**: [Home](../intro.md)