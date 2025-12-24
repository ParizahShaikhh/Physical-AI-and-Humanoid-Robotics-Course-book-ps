---
sidebar_label: 'Chapter 4: Services and Actions'
sidebar_position: 4
title: 'Chapter 4: Services and Actions – Synchronous and Long-Running Operations'
description: 'Understanding request-response communication and goal-oriented interactions in ROS 2'
---

# Chapter 4: Services and Actions – Synchronous and Long-Running Operations

## Services: Request-Response Communication

Services in ROS 2 implement a synchronous request-response communication pattern. Unlike topics which are asynchronous, services establish a direct connection between a client and a server for immediate request-response interaction.

A service consists of:
- **Service Server**: Waits for requests and sends responses
- **Service Client**: Sends requests and waits for responses
- **Service Interface**: Defines the request and response message types

## Service Interface Definition

Service interfaces are defined in `.srv` files that specify both request and response message structures:

```
# Request part (before the --- separator)
string name
int32 age
---
# Response part (after the --- separator)
bool success
string message
```

## When to Use Services

Services are appropriate for:
- Operations that require immediate responses
- Configuration changes that need acknowledgment
- Query operations (e.g., getting robot status)
- Operations that should complete quickly
- Commands that need confirmation of success/failure

## Actions: Goal-Oriented Communication

Actions extend the request-response pattern to support long-running operations with feedback and status updates. An action consists of three message types:
- **Goal**: Specifies what the action should accomplish
- **Feedback**: Provides periodic updates on progress
- **Result**: Contains the final outcome of the action

## Action Flow

The action communication pattern follows this sequence:
1. Client sends a goal to the action server
2. Server acknowledges the goal and begins execution
3. Server periodically sends feedback about progress
4. Client can monitor progress and cancel if needed
5. Server sends the final result when complete

## Use Cases in Humanoid Robotics

**Services** are ideal for:
- Calibration routines
- Emergency stop activation
- Parameter queries
- Simple movement commands
- System status checks

**Actions** are perfect for:
- Navigation to waypoints
- Manipulation tasks with grasping
- Walking gaits and locomotion
- Complex motion sequences
- Any operation that takes significant time and needs monitoring

## Comparison of Communication Patterns

| Pattern | Type | Use Case |
|---------|------|----------|
| Topics | Asynchronous | Continuous data streams, sensor data |
| Services | Synchronous | Quick request-response operations |
| Actions | Asynchronous with status | Long-running operations with feedback |

## Quality of Service for Services and Actions

Like topics, services and actions support QoS settings for reliability, durability, and other communication properties. This ensures appropriate behavior for different types of operations.

## Summary

Services and actions complement the topic-based communication by providing synchronous request-response patterns and goal-oriented operations with feedback. Together with topics, they form the complete set of communication primitives needed for complex robotic systems.