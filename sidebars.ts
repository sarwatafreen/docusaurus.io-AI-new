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
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1 – ROS 2: The Robot Nervous System (Weeks 1-4)',
      collapsed: false,
      items: [
        'Module-1/index',
        'Module-1/Lesson-1-Your-First-Robot',
        'Module-1/Lesson-2-Move-Robot-Keyboard',
        'Module-1/Lesson-3-Talk-to-Robot-Parts',
        'Module-1/Lesson-4-Build-Humanoid-Body'
      ],
    },
    {
      type: 'category',
      label: 'Module 2 – Simulation: Gazebo & Physics (Weeks 5-7)',
      collapsed: false,
      items: [
        'Module-2/index',
        'Module-2/Lesson-1-Spawn-Robot-Gazebo',
        'Module-2/Lesson-2-Make-It-Fall-Walk-Get-Up',
        'Module-2/Lesson-3-Add-Real-Sensors',
        'Module-2/Lesson-4-See-Robot-Unity'
      ],
    },
    {
      type: 'category',
      label: 'Module 3 – The AI Brain: NVIDIA Isaac Sim (Weeks 8-10)',
      collapsed: false,
      items: [
        'Module-3/index',
        'Module-3/Lesson-1-Run-Photorealistic-Isaac-Sim',
        'Module-3/Lesson-2-Train-robot-with-pictures',
        'Module-3/Lesson-3-VSLAM-room-mapping',
        'Module-3/Lesson-4-Navigation-with-Nav2'
      ],
    },
    {
      type: 'category',
      label: 'Module 4 – Vision-Language-Action Magic (Weeks 11-13)',
      collapsed: false,
      items: [
        'Module-4/index',
        'Module-4/Lesson-1-Voice-Recognition-Integration',
        'Module-4/Lesson-2-GPT-to-Robot-Actions',
        'Module-4/Lesson-3-Final-Project-Voice-Controlled-Humanoid',
        'Module-4/Lesson-4-Deployment-and-Production'
      ],
    },
    {
      type: 'category',
      label: 'Hardware & Setup Guide',
      collapsed: false,
      items: [
        'hardware-setup',
        'docker-setup',
        'troubleshooting'
      ],
    }
  ],
};

export default sidebars;
