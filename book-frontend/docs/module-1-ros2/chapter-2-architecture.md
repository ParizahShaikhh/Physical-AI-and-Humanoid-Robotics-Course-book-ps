---
sidebar_label: 'Chapter 2: Architecture'
sidebar_position: 2
title: 'Chapter 2: ROS 2 Architecture and Core Concepts'
description: 'Understanding nodes, executors, and DDS in ROS 2'
---

# Chapter 2: ROS 2 Architecture and Core Concepts

## Nodes and Executors

In ROS 2, a node is an executable that uses the ROS client library to communicate with other nodes. Nodes are the fundamental building blocks of a ROS system, each typically responsible for a specific task or functionality.

An executor manages the execution of one or more nodes, handling the execution of callbacks for various ROS entities like subscriptions, services, and timers. The executor abstracts the complexity of multithreaded execution.

## DDS (Data Distribution Service)

DDS is the middleware layer that ROS 2 uses for communication. It provides a standardized interface for real-time, scalable, and reliable data exchange between distributed applications.

Key features of DDS:
- Data-centric publish-subscribe model
- Quality of Service (QoS) policies for different communication needs
- Built-in discovery mechanisms
- Language and platform independence

## Computation Graph Overview

The ROS 2 computation graph represents the network of nodes and their communication patterns. This includes:

- **Nodes**: Individual processes that perform computation
- **Topics**: Named buses over which nodes exchange messages
- **Services**: Synchronous request/response communication
- **Actions**: Goal-oriented communication with feedback
- **Parameters**: Configuration values that can be shared between nodes

## Real-time and Distributed Design Principles

ROS 2 is designed with real-time and distributed systems in mind:

- **Real-time capabilities**: Support for real-time operating systems and deterministic behavior
- **Distributed architecture**: Nodes can run on different machines and communicate seamlessly
- **Fault tolerance**: Mechanisms to handle node failures and network partitions
- **Scalability**: Ability to add nodes and functionality without disrupting existing systems

## Summary

Understanding the architecture is crucial for effectively designing and implementing ROS 2 systems. The next chapter will focus on the most common communication pattern: topics.