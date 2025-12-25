// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Tutorial',
      items: [
        'intro',
        {
          type: 'category',
          label: 'Module 1: ROS 2 Nervous System',
          items: [
            'module-1-ros2-nervous-system/introduction-to-ros2',
            'module-1-ros2-nervous-system/communication-model',
            'module-1-ros2-nervous-system/robot-structure-urdf',
          ],
        },
        {
          type: 'category',
          label: 'Module 2: Digital Twin',
          items: [
            'module-2-digital-twin/digital-twins-hri-unity',
            'module-2-digital-twin/physics-simulation',
            'module-2-digital-twin/sensor-simulation-validation',
          ],
        },
        {
          type: 'category',
          label: 'Module 3: AI Robot Brain',
          items: [
            'Module-3-AI-Robot-Brain/chapter-1-nvidia-isaac-sim',
            'Module-3-AI-Robot-Brain/chapter-2-isaac-ros-vslam-navigation',
            'Module-3-AI-Robot-Brain/chapter-3-nav2-path-planning',
          ],
        },
        {
          type: 'category',
          label: 'Module 4: VLA Robotics',
          items: [
            'module-4-vla-robotics/voice-to-action-whisper',
            'module-4-vla-robotics/cognitive-planning-llms',
            'module-4-vla-robotics/capstone-autonomous-humanoid',
          ],
        },
      ],
    },
  ],
};

module.exports = sidebars;