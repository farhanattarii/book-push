---
sidebar_position: 1
title: 'Introduction to ROS 2 for Physical AI'
---

# Introduction to ROS 2 for Physical AI

## What is ROS 2?

ROS 2 (Robot Operating System 2) is not an operating system in the traditional sense, but rather a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

For humanoid robots, ROS 2 serves as the middleware nervous system, enabling different components of the robot to communicate effectively with each other. This is particularly important for humanoid robots which have complex sensorimotor systems that need to work in coordination.

### Key Features of ROS 2

- **Distributed Computing**: ROS 2 allows different parts of a robot system to run on different computers or processors
- **Language Independence**: Supports multiple programming languages (C++, Python, Rust, etc.)
- **Real-time Support**: Enhanced real-time capabilities for time-critical applications
- **Security**: Built-in security features for safe robot operation
- **Middleware Abstraction**: Uses DDS (Data Distribution Service) as the underlying communication layer

## Why ROS 2 Matters for Humanoid Robotics

Humanoid robots present unique challenges that make ROS 2 particularly valuable:

### Complexity Management
Humanoid robots have dozens of joints, multiple sensors (cameras, IMUs, force/torque sensors), and complex control systems. ROS 2's modular architecture allows these components to be developed and tested independently before integration.

### Sensor Integration
Humanoid robots typically have:
- Multiple cameras for vision processing
- IMUs for balance and orientation
- Force/torque sensors in joints
- Tactile sensors in hands and feet
- Microphones for audio processing

ROS 2 provides standardized message types and communication patterns for all these sensors.

### Control Architecture
Humanoid robots require sophisticated control systems:
- Low-level joint controllers
- Balance and locomotion controllers
- High-level behavior planners
- AI perception and decision-making systems

ROS 2 enables these different control layers to communicate effectively.

## DDS Concepts: The Foundation of ROS 2 Communication

DDS (Data Distribution Service) is the middleware technology that underlies ROS 2. Understanding DDS concepts is crucial for effectively using ROS 2 in humanoid robotics.

### Data-Centric Architecture
Unlike traditional request-response systems, DDS uses a data-centric approach where:
- Data is the central concept rather than services or functions
- Publishers and subscribers are decoupled in time and space
- The middleware handles data distribution automatically

### Quality of Service (QoS) Settings
DDS provides rich QoS settings that are particularly important for humanoid robots:

- **Reliability**: Ensures critical control messages are delivered
- **Durability**: Maintains message history for late-joining nodes
- **Deadline**: Ensures messages meet timing constraints
- **Liveliness**: Monitors node availability for safety
- **History**: Controls how many samples are kept

### Topics, Publishers, and Subscribers
The fundamental communication pattern in ROS 2 (based on DDS):

- **Topics**: Named buses over which nodes exchange messages
- **Publishers**: Nodes that send messages to topics
- **Subscribers**: Nodes that receive messages from topics

This publish-subscribe pattern is ideal for humanoid robots where multiple components need to share sensor data and control commands.

## ROS 2 vs ROS 1: Key Differences for Humanoid Robotics

ROS 2 addresses several limitations of ROS 1 that were particularly problematic for humanoid robots:

### Real-time Performance
ROS 1 had limited real-time capabilities, which is critical for humanoid balance and control. ROS 2 provides better real-time support.

### Security
ROS 1 had no built-in security, making it unsuitable for robots that need to operate in secure environments. ROS 2 includes comprehensive security features.

### Multi-robot Systems
ROS 2 makes it much easier to coordinate multiple robots, which is relevant for humanoid robot teams or human-robot interaction scenarios.

## Getting Started with ROS 2 for Humanoid Robotics

### Installation
ROS 2 supports multiple distributions. For humanoid robotics, it's recommended to use the latest Long-Term Support (LTS) distribution.

### Basic Concepts
- **Nodes**: Individual processes that perform computation
- **Packages**: Collections of related files and executables
- **Workspaces**: Directories where you modify and build code
- **Launch files**: Configuration files to start multiple nodes at once

### Common Tools
- **ros2 run**: Execute a node from a package
- **ros2 topic**: Work with topics (publish, subscribe, echo)
- **ros2 service**: Work with services
- **rqt**: GUI tools for visualizing robot state
- **RViz**: 3D visualization tool for robot data

## Summary

This chapter introduced you to ROS 2 as the middleware nervous system for humanoid robots. You learned about:

- What ROS 2 is and its key features
- Why ROS 2 is particularly valuable for humanoid robotics
- The fundamental DDS concepts that underlie ROS 2 communication
- Key differences between ROS 1 and ROS 2
- Basic tools and concepts to get started

## Navigation

- **Next**: [ROS 2 Communication Model](./communication-model.md) - Dive deeper into nodes, topics, services, and how they enable the agent ↔ controller flow essential for humanoid robotics
- **Previous**: [Home](../intro.md)