/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.
 */

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  courseSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Introduction to Physical AI',
      link: {
        type: 'doc',
        id: 'introduction/index',
      },
      collapsed: false,
      items: [
        'introduction/week-01',
        'introduction/week-02',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      link: {
        type: 'doc',
        id: 'ros2/index',
      },
      collapsed: false,
      items: [
        'ros2/week-03',
        'ros2/week-04',
        'ros2/week-05',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Gazebo & Unity Simulation',
      link: {
        type: 'doc',
        id: 'simulation/index',
      },
      collapsed: false,
      items: [
        'simulation/week-06',
        'simulation/week-07',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac Platform',
      link: {
        type: 'doc',
        id: 'isaac/index',
      },
      collapsed: false,
      items: [
        'isaac/week-08',
        'isaac/week-09',
        'isaac/week-10',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA & Capstone',
      link: {
        type: 'doc',
        id: 'vla/index',
      },
      collapsed: false,
      items: [
        'vla/week-11',
        'vla/week-12',
        'vla/week-13',
      ],
    },
    {
      type: 'category',
      label: 'Assessments',
      link: {
        type: 'doc',
        id: 'assessments/index',
      },
      collapsed: true,
      items: [],
    },
  ],
};

module.exports = sidebars;
