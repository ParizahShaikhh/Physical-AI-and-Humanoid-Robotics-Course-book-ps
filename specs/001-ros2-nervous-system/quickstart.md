# Quickstart Guide: Setting up Docusaurus for ROS 2 Educational Content

## Overview
This guide provides step-by-step instructions to set up the Docusaurus documentation site for the "Robotic Nervous System (ROS 2)" module. This will create a comprehensive educational resource for students learning ROS 2 concepts.

## Prerequisites
- Node.js v18 or higher
- npm or yarn package manager
- Git
- A text editor or IDE

## Step 1: Initialize Docusaurus Project

1. Create a new Docusaurus project:
   ```bash
   npx create-docusaurus@latest website classic
   ```

2. Navigate to the project directory:
   ```bash
   cd website
   ```

## Step 2: Install Additional Dependencies

1. Install any additional dependencies needed for educational content:
   ```bash
   npm install @docusaurus/module-type-aliases @docusaurus/types
   ```

## Step 3: Configure Site Metadata

1. Edit `docusaurus.config.js` to update site configuration:
   ```js
   // docusaurus.config.js
   import {themes as prismThemes} from 'prism-react-renderer';

   /** @type {import('@docusaurus/types').Config} */
   const config = {
     title: 'Physical AI and Humanoid Robotics Course',
     tagline: 'Learning ROS 2 as the Robotic Nervous System',
     favicon: 'img/favicon.ico',

     // Set the production url of your site here
     url: 'https://your-username.github.io',
     // Set the /<base>/ pathname under which your site is served
     // For GitHub Pages, this is usually '/<project-name>/'
     baseUrl: '/Physical-AI-and-Humanoid-Robotics-Course/',

     // GitHub pages deployment config
     organizationName: 'your-username',
     projectName: 'Physical-AI-and-Humanoid-Robotics-Course',
     deploymentBranch: 'gh-pages',

     onBrokenLinks: 'throw',
     onBrokenMarkdownLinks: 'warn',

     // Even if you don't use internationalization, you can use this field to set
     // useful metadata like html lang. For example, if your site is Chinese, you
     // may want to replace "en" with "zh-Hans".
     i18n: {
       defaultLocale: 'en',
       locales: ['en'],
     },

     presets: [
       [
         'classic',
         /** @type {import('@docusaurus/preset-classic').Options} */
         ({
           docs: {
             sidebarPath: require.resolve('./sidebars.js'),
             // Please change this to your repo.
             // Remove this to remove the "edit this page" links.
             editUrl:
               'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
           },
           blog: false, // Disable blog if not needed
           theme: {
             customCss: require.resolve('./src/css/custom.css'),
           },
         }),
       ],
     ],

     themeConfig:
       /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
       ({
         // Replace with your project's social card
         image: 'img/docusaurus-social-card.jpg',
         navbar: {
           title: 'ROS 2 Robotics Course',
           logo: {
             alt: 'My Site Logo',
             src: 'img/logo.svg',
           },
           items: [
             {
               type: 'docSidebar',
               sidebarId: 'tutorialSidebar',
               position: 'left',
               label: 'Module 1',
             },
             {
               href: 'https://github.com/facebook/docusaurus',
               label: 'GitHub',
               position: 'right',
             },
           ],
         },
         footer: {
           style: 'dark',
           links: [
             {
               title: 'Modules',
               items: [
                 {
                   label: 'Module 1: ROS 2',
                   to: '/docs/module-1-ros2',
                 },
               ],
             },
             {
               title: 'Community',
               items: [
                 {
                   label: 'Stack Overflow',
                   href: 'https://stackoverflow.com/questions/tagged/docusaurus',
                 },
                 {
                   label: 'Discord',
                   href: 'https://discordapp.com/invite/docusaurus',
                 },
               ],
             },
             {
               title: 'More',
               items: [
                 {
                   label: 'GitHub',
                   href: 'https://github.com/facebook/docusaurus',
                 },
               ],
             },
           ],
           copyright: `Copyright © ${new Date().getFullYear()} Physical AI and Humanoid Robotics Course. Built with Docusaurus.`,
         },
         prism: {
           theme: prismThemes.github,
           darkTheme: prismThemes.dracula,
         },
       }),
   };

   module.exports = config;
   ```

## Step 4: Create Module Directory Structure

1. Create the directory structure for the ROS 2 module:
   ```bash
   mkdir -p docs/module-1-ros2
   ```

## Step 5: Create Chapter Files

1. Create the 7 chapter files in the `docs/module-1-ros2/` directory:

   **docs/module-1-ros2/index.md:**
   ```md
   # Module 1: The Robotic Nervous System (ROS 2)

   Welcome to Module 1 of the Physical AI and Humanoid Robotics Course. This module introduces you to ROS 2 as the core middleware for humanoid robot control, enabling communication between AI agents (digital brain) and robotic actuators/sensors (physical body).

   ## Learning Objectives

   By the end of this module, you should be able to:
   - Explain ROS 2 architecture and communication primitives
   - Build and run ROS 2 nodes using Python (rclpy)
   - Connect AI logic to robot controllers via ROS 2
   - Read, modify, and reason about humanoid URDF models

   ## Prerequisites

   - Basic Python knowledge
   - Fundamental understanding of AI concepts
   - Interest in robotics and physical AI

   ## Chapter Overview

   1. [Introduction to Physical AI and ROS 2](./chapter-1-introduction)
   2. [ROS 2 Architecture and Core Concepts](./chapter-2-architecture)
   3. [Topics – Asynchronous Robot Communication](./chapter-3-topics)
   4. [Services and Actions](./chapter-4-services-actions)
   5. [Python Agents with rclpy](./chapter-5-python-agents)
   6. [Humanoid Robot Modeling with URDF](./chapter-6-urdf-modeling)
   7. [From AI Brain to Robot Body](./chapter-7-end-to-end)
   ```

   **docs/module-1-ros2/chapter-1-introduction.md:**
   ```md
   ---
   sidebar_label: 'Chapter 1: Introduction'
   sidebar_position: 1
   title: 'Chapter 1: Introduction to Physical AI and ROS 2'
   description: 'Understanding the role of ROS 2 in embodied intelligence'
   ---

   # Chapter 1: Introduction to Physical AI and ROS 2

   ## Role of ROS 2 in Embodied Intelligence

   ROS 2 (Robot Operating System 2) serves as the nervous system of robots, providing the middleware infrastructure that enables communication between different components of a robotic system. Unlike traditional software systems, robots require real-time communication between sensors, actuators, and processing units that may be distributed across different hardware platforms.

   In the context of embodied intelligence, where AI agents need to interact with the physical world through robotic bodies, ROS 2 provides the essential communication layer that connects the "digital brain" (AI algorithms) with the "physical body" (sensors and actuators).

   ## Why Middleware is the "Nervous System" of Robots

   Just as the biological nervous system transmits signals between the brain and the body, ROS 2 middleware facilitates communication between AI decision-making components and the physical robotic systems. This communication must be:
   - Real-time capable
   - Distributed across multiple nodes
   - Fault-tolerant
   - Scalable

   ## ROS 2 vs ROS 1: Conceptual Differences

   While a detailed migration guide is beyond the scope of this module, it's important to understand the key conceptual differences:

   - **Architecture**: ROS 2 uses DDS (Data Distribution Service) for communication, providing better real-time performance and distributed system capabilities
   - **Security**: ROS 2 includes built-in security features that were not present in ROS 1
   - **Quality of Service (QoS)**: ROS 2 provides more sophisticated control over message delivery guarantees
   - **Lifecycle management**: More robust node lifecycle management in ROS 2

   ## Summary

   This chapter introduced the fundamental concepts of ROS 2 as the nervous system of robots. In the next chapter, we'll dive deeper into the architecture and core concepts of ROS 2.
   ```

   **docs/module-1-ros2/chapter-2-architecture.md:**
   ```md
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
   ```

   [Additional chapters would follow the same pattern, but for brevity I'll stop here as the structure is clear]

## Step 6: Configure Sidebar Navigation

1. Update `sidebars.js` to create the sidebar structure:
   ```js
   // sidebars.js
   /** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
   const sidebars = {
     tutorialSidebar: [
       {
         type: 'autogenerated',
         dirName: '.',
       },
     ],
     module1Sidebar: [
       {
         type: 'category',
         label: 'Module 1: The Robotic Nervous System (ROS 2)',
         items: [
           'module-1-ros2/index',
           'module-1-ros2/chapter-1-introduction',
           'module-1-ros2/chapter-2-architecture',
           'module-1-ros2/chapter-3-topics',
           'module-1-ros2/chapter-4-services-actions',
           'module-1-ros2/chapter-5-python-agents',
           'module-1-ros2/chapter-6-urdf-modeling',
           'module-1-ros2/chapter-7-end-to-end',
         ],
       },
     ],
   };

   module.exports = sidebars;
   ```

## Step 7: Run the Development Server

1. Start the development server:
   ```bash
   npm run start
   ```

2. Your site will be accessible at `http://localhost:3000`

## Step 8: Build for Production

1. To build the site for deployment:
   ```bash
   npm run build
   ```

2. The built site will be in the `build/` directory

## Step 9: Deploy to GitHub Pages

1. To deploy to GitHub Pages:
   ```bash
   GIT_USER=<Your GitHub username> npm run deploy
   ```

## Next Steps

1. Complete all 7 chapters with detailed content
2. Add code examples and diagrams
3. Create practical exercises for each chapter
4. Implement assessment questions
5. Test the navigation and user experience
6. Deploy to GitHub Pages for public access

## Troubleshooting

- If you encounter issues with the development server, try clearing the cache: `npm run clear`
- Make sure Node.js version meets the requirements (v18+)
- Verify all file paths are correct in the sidebar configuration