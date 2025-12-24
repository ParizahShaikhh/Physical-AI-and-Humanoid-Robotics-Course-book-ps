import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
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
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2-digital-twin/index',
        'module-2-digital-twin/chapter-1-digital-twins',
        'module-2-digital-twin/chapter-2-gazebo-architecture',
        'module-2-digital-twin/chapter-3-physics-simulation',
        'module-2-digital-twin/chapter-4-ros2-integration',
        'module-2-digital-twin/chapter-5-unity-interaction',
        'module-2-digital-twin/chapter-6-simulated-sensors',
        'module-2-digital-twin/chapter-7-ai-training',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module-3-ai-robot-brain/index',
        'module-3-ai-robot-brain/chapter-1-simulation-intelligence',
        'module-3-ai-robot-brain/chapter-2-isaac-platform',
        'module-3-ai-robot-brain/chapter-3-isaac-sim-worlds',
        'module-3-ai-robot-brain/chapter-4-synthetic-data',
        'module-3-ai-robot-brain/chapter-5-isaac-ros-perception',
        'module-3-ai-robot-brain/chapter-6-vslam-navigation',
        'module-3-ai-robot-brain/chapter-7-vla-systems',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4-vla-systems/index',
        'module-4-vla-systems/chapter-1-vla-physical-ai',
        'module-4-vla-systems/chapter-2-speech-to-text',
        'module-4-vla-systems/chapter-3-language-understanding',
        'module-4-vla-systems/chapter-4-llm-planning',
        'module-4-vla-systems/chapter-5-vision-action',
        'module-4-vla-systems/chapter-6-ros2-orchestration',
        'module-4-vla-systems/chapter-7-capstone',
      ],
    },
    {
      type: 'category',
      label: 'Module 5: Deployment, Integration, and Real-World Humanoids',
      items: [
        'module-5-real-world-humanoids/index',
        'module-5-real-world-humanoids/chapter-1-simulation-reality',
        'module-5-real-world-humanoids/chapter-2-sim-to-real-transfer',
        'module-5-real-world-humanoids/chapter-3-system-integration',
        'module-5-real-world-humanoids/chapter-4-safety-human-interaction',
        'module-5-real-world-humanoids/chapter-5-testing-validation',
        'module-5-real-world-humanoids/chapter-6-performance-evaluation',
        'module-5-real-world-humanoids/chapter-7-deployment-maintenance',
      ],
    },
    {
      type: 'category',
      label: 'Module 6: Advanced Topics, Optimization, and Future Directions',
      items: [
        'module-6-advanced-topics/index',
        'module-6-advanced-topics/chapter-1-scaling-physical-ai',
        'module-6-advanced-topics/chapter-2-performance-optimization',
        'module-6-advanced-topics/chapter-3-human-robot-collaboration',
        'module-6-advanced-topics/chapter-4-continuous-learning',
        'module-6-advanced-topics/chapter-5-reliability-governance',
        'module-6-advanced-topics/chapter-6-emerging-trends',
        'module-6-advanced-topics/chapter-7-future-embodied',
      ],
    },
    {
      type: 'category',
      label: 'Module 7: Capstone, Evaluation, and Professional Practice',
      items: [
        'module-7-capstone-eval/index',
        'module-7-capstone-eval/chapter-1-capstone-overview',
        'module-7-capstone-eval/chapter-2-system-architecture',
        'module-7-capstone-eval/chapter-3-end-to-end-pipeline',
        'module-7-capstone-eval/chapter-4-evaluation-metrics',
        'module-7-capstone-eval/chapter-5-failure-modes',
        'module-7-capstone-eval/chapter-6-documentation-demos',
        'module-7-capstone-eval/chapter-7-real-world-applications',
      ],
    },
  ],
};

export default sidebars;
