---
sidebar_label: 'Chapter 3: Topics Communication'
sidebar_position: 3
title: 'Chapter 3: Topics – Asynchronous Robot Communication'
description: 'Understanding publishers, subscribers, and message flow in ROS 2'
---

# Chapter 3: Topics – Asynchronous Robot Communication

## Publishers and Subscribers

In ROS 2, topics enable asynchronous communication between nodes using a publish-subscribe pattern. Publishers send messages to a topic, and subscribers receive messages from that topic. This decouples the sender and receiver in time and space, allowing for flexible and robust communication patterns.

A publisher node sends data to a topic without knowing which (or if any) subscriber nodes will receive it. Similarly, a subscriber node receives data from a topic without knowing which publisher sent it.

## Message Types and Data Flow

Messages in ROS 2 are defined using `.msg` files that specify the data structure. Common message types include:
- `std_msgs`: Basic data types like integers, floats, strings
- `sensor_msgs`: Sensor data like images, laser scans, IMU data
- `geometry_msgs`: Geometric primitives like poses, points, vectors
- `nav_msgs`: Navigation-related messages like paths and odometry

The data flow in a topic-based system is:
1. Publisher creates and sends messages to a topic
2. DDS middleware handles message delivery
3. Subscribers receive messages from the topic
4. Subscriber processes the received data

## Use Cases in Humanoid Sensing and Control

Topics are particularly useful for:
- **Sensor data distribution**: Multiple nodes can subscribe to sensor data simultaneously
- **Control commands**: Sending motor commands or joint positions
- **Status updates**: Broadcasting robot state information
- **Event notifications**: Signaling when specific conditions occur

In humanoid robotics, topics enable the distribution of sensor data (e.g., camera feeds, IMU readings, joint states) to multiple processing nodes simultaneously, and allow control nodes to send commands to actuators without tight coupling.

## Summary

Topics form the backbone of most ROS 2 communication patterns, enabling flexible and decoupled system architectures. The next chapter will cover services and actions for synchronous and long-running operations.