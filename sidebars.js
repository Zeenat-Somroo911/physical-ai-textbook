/**
 * Sidebar configuration for Physical AI & Humanoid Robotics Textbook
 * Organized into 4 main modules plus projects
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    'getting-started',
    'resources',
    'faq',
    'glossary',
    {
      type: 'category',
      label: 'Module 1: ROS2 Fundamentals',
      collapsed: false,
      items: [
        'module-01-ros2/intro',
        'module-01-ros2/introduction',
        'module-01-ros2/nodes-topics',
        'module-01-ros2/services-actions',
        'module-01-ros2/python-rclpy',
        'module-01-ros2/urdf-basics',
        'module-01-ros2/launch-files',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation & Gazebo',
      collapsed: false,
      items: [
        'module-02-simulation/intro',
        'module-02-simulation/gazebo-intro',
        'module-02-simulation/urdf-sdf',
        'module-02-simulation/physics-simulation',
        'module-02-simulation/sensor-simulation',
        'module-02-simulation/unity-integration',
        'module-02-simulation/practical-lab',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: AI-Powered Perception',
      collapsed: false,
      items: [
        'module-03-isaac/intro',
        'module-03-isaac/free-alternatives',
        'module-03-isaac/pybullet-basics',
        'module-03-isaac/computer-vision',
        'module-03-isaac/slam-basics',
        'module-03-isaac/navigation',
        'module-03-isaac/perception-pipeline',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      collapsed: false,
      items: [
        'module-04-vla/intro',
        'module-04-vla/vla-introduction',
        'module-04-vla/voice-commands',
        'module-04-vla/llm-integration',
        'module-04-vla/action-planning',
        'module-04-vla/multimodal',
        'module-04-vla/capstone-project',
      ],
    },
    {
      type: 'category',
      label: 'Projects',
      collapsed: false,
      items: [
        'projects/overview',
        'projects/project-01-ros2-basics',
        'projects/project-02-simulation',
        'projects/project-03-perception',
        'projects/project-04-vla',
        'projects/final-project',
      ],
    },
    {
      type: 'category',
      label: 'Development',
      collapsed: true,
      items: [
        'development/subagents',
      ],
    },
  ],
};

module.exports = sidebars;

