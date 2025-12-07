import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/ai-native-book/__docusaurus/debug',
    component: ComponentCreator('/ai-native-book/__docusaurus/debug', 'bf0'),
    exact: true
  },
  {
    path: '/ai-native-book/__docusaurus/debug/config',
    component: ComponentCreator('/ai-native-book/__docusaurus/debug/config', '587'),
    exact: true
  },
  {
    path: '/ai-native-book/__docusaurus/debug/content',
    component: ComponentCreator('/ai-native-book/__docusaurus/debug/content', '318'),
    exact: true
  },
  {
    path: '/ai-native-book/__docusaurus/debug/globalData',
    component: ComponentCreator('/ai-native-book/__docusaurus/debug/globalData', '757'),
    exact: true
  },
  {
    path: '/ai-native-book/__docusaurus/debug/metadata',
    component: ComponentCreator('/ai-native-book/__docusaurus/debug/metadata', 'f6e'),
    exact: true
  },
  {
    path: '/ai-native-book/__docusaurus/debug/registry',
    component: ComponentCreator('/ai-native-book/__docusaurus/debug/registry', 'e58'),
    exact: true
  },
  {
    path: '/ai-native-book/__docusaurus/debug/routes',
    component: ComponentCreator('/ai-native-book/__docusaurus/debug/routes', 'adc'),
    exact: true
  },
  {
    path: '/ai-native-book/blog',
    component: ComponentCreator('/ai-native-book/blog', 'ab0'),
    exact: true
  },
  {
    path: '/ai-native-book/blog/archive',
    component: ComponentCreator('/ai-native-book/blog/archive', 'd97'),
    exact: true
  },
  {
    path: '/ai-native-book/blog/authors',
    component: ComponentCreator('/ai-native-book/blog/authors', '358'),
    exact: true
  },
  {
    path: '/ai-native-book/blog/authors/all-sebastien-lorber-articles',
    component: ComponentCreator('/ai-native-book/blog/authors/all-sebastien-lorber-articles', 'e50'),
    exact: true
  },
  {
    path: '/ai-native-book/blog/authors/yangshun',
    component: ComponentCreator('/ai-native-book/blog/authors/yangshun', '952'),
    exact: true
  },
  {
    path: '/ai-native-book/blog/first-blog-post',
    component: ComponentCreator('/ai-native-book/blog/first-blog-post', 'f3a'),
    exact: true
  },
  {
    path: '/ai-native-book/blog/long-blog-post',
    component: ComponentCreator('/ai-native-book/blog/long-blog-post', 'add'),
    exact: true
  },
  {
    path: '/ai-native-book/blog/mdx-blog-post',
    component: ComponentCreator('/ai-native-book/blog/mdx-blog-post', '673'),
    exact: true
  },
  {
    path: '/ai-native-book/blog/tags',
    component: ComponentCreator('/ai-native-book/blog/tags', 'c39'),
    exact: true
  },
  {
    path: '/ai-native-book/blog/tags/docusaurus',
    component: ComponentCreator('/ai-native-book/blog/tags/docusaurus', '5c7'),
    exact: true
  },
  {
    path: '/ai-native-book/blog/tags/facebook',
    component: ComponentCreator('/ai-native-book/blog/tags/facebook', '455'),
    exact: true
  },
  {
    path: '/ai-native-book/blog/tags/hello',
    component: ComponentCreator('/ai-native-book/blog/tags/hello', 'b4e'),
    exact: true
  },
  {
    path: '/ai-native-book/blog/tags/hola',
    component: ComponentCreator('/ai-native-book/blog/tags/hola', 'c16'),
    exact: true
  },
  {
    path: '/ai-native-book/blog/welcome',
    component: ComponentCreator('/ai-native-book/blog/welcome', 'bcf'),
    exact: true
  },
  {
    path: '/ai-native-book/markdown-page',
    component: ComponentCreator('/ai-native-book/markdown-page', 'a10'),
    exact: true
  },
  {
    path: '/ai-native-book/docs',
    component: ComponentCreator('/ai-native-book/docs', '36f'),
    routes: [
      {
        path: '/ai-native-book/docs',
        component: ComponentCreator('/ai-native-book/docs', '934'),
        routes: [
          {
            path: '/ai-native-book/docs',
            component: ComponentCreator('/ai-native-book/docs', '591'),
            routes: [
              {
                path: '/ai-native-book/docs/ai-robot-brain/',
                component: ComponentCreator('/ai-native-book/docs/ai-robot-brain/', 'a60'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/ai-native-book/docs/ai-robot-brain/isaac-ros-perception',
                component: ComponentCreator('/ai-native-book/docs/ai-robot-brain/isaac-ros-perception', 'a7e'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/ai-native-book/docs/ai-robot-brain/nav2-path-planning',
                component: ComponentCreator('/ai-native-book/docs/ai-robot-brain/nav2-path-planning', '5ad'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/ai-native-book/docs/ai-robot-brain/nvidia-isaac-sim',
                component: ComponentCreator('/ai-native-book/docs/ai-robot-brain/nvidia-isaac-sim', '3be'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/ai-native-book/docs/appendices/cloud-setup',
                component: ComponentCreator('/ai-native-book/docs/appendices/cloud-setup', '111'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/ai-native-book/docs/appendices/hardware-guide',
                component: ComponentCreator('/ai-native-book/docs/appendices/hardware-guide', 'f03'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/ai-native-book/docs/appendices/resources',
                component: ComponentCreator('/ai-native-book/docs/appendices/resources', 'c16'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/ai-native-book/docs/appendices/troubleshooting',
                component: ComponentCreator('/ai-native-book/docs/appendices/troubleshooting', '23d'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/ai-native-book/docs/digital-twin/',
                component: ComponentCreator('/ai-native-book/docs/digital-twin/', 'c81'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/ai-native-book/docs/digital-twin/gazebo-fundamentals',
                component: ComponentCreator('/ai-native-book/docs/digital-twin/gazebo-fundamentals', 'ac1'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/ai-native-book/docs/digital-twin/physics-simulation',
                component: ComponentCreator('/ai-native-book/docs/digital-twin/physics-simulation', 'e93'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/ai-native-book/docs/digital-twin/unity-integration',
                component: ComponentCreator('/ai-native-book/docs/digital-twin/unity-integration', 'ebf'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/ai-native-book/docs/glossary',
                component: ComponentCreator('/ai-native-book/docs/glossary', '96f'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/ai-native-book/docs/instructor-guide',
                component: ComponentCreator('/ai-native-book/docs/instructor-guide', '7dc'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/ai-native-book/docs/intro',
                component: ComponentCreator('/ai-native-book/docs/intro', '0e8'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/ai-native-book/docs/robotic-nervous-system/',
                component: ComponentCreator('/ai-native-book/docs/robotic-nervous-system/', 'c1e'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/ai-native-book/docs/robotic-nervous-system/introduction-physical-ai',
                component: ComponentCreator('/ai-native-book/docs/robotic-nervous-system/introduction-physical-ai', 'b35'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/ai-native-book/docs/robotic-nervous-system/ros2-architecture',
                component: ComponentCreator('/ai-native-book/docs/robotic-nervous-system/ros2-architecture', '661'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/ai-native-book/docs/robotic-nervous-system/ros2-python-development',
                component: ComponentCreator('/ai-native-book/docs/robotic-nervous-system/ros2-python-development', '51b'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/ai-native-book/docs/robotic-nervous-system/urdf-robot-modeling',
                component: ComponentCreator('/ai-native-book/docs/robotic-nervous-system/urdf-robot-modeling', '19f'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/ai-native-book/docs/student-resources/ros2_python_cheat_sheet',
                component: ComponentCreator('/ai-native-book/docs/student-resources/ros2_python_cheat_sheet', '492'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/ai-native-book/docs/student-resources/vla_reference_card',
                component: ComponentCreator('/ai-native-book/docs/student-resources/vla_reference_card', 'd31'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/ai-native-book/docs/tutorial-basics/congratulations',
                component: ComponentCreator('/ai-native-book/docs/tutorial-basics/congratulations', 'bf8'),
                exact: true
              },
              {
                path: '/ai-native-book/docs/tutorial-basics/create-a-blog-post',
                component: ComponentCreator('/ai-native-book/docs/tutorial-basics/create-a-blog-post', '3af'),
                exact: true
              },
              {
                path: '/ai-native-book/docs/tutorial-basics/create-a-document',
                component: ComponentCreator('/ai-native-book/docs/tutorial-basics/create-a-document', 'd24'),
                exact: true
              },
              {
                path: '/ai-native-book/docs/tutorial-basics/create-a-page',
                component: ComponentCreator('/ai-native-book/docs/tutorial-basics/create-a-page', '917'),
                exact: true
              },
              {
                path: '/ai-native-book/docs/tutorial-basics/deploy-your-site',
                component: ComponentCreator('/ai-native-book/docs/tutorial-basics/deploy-your-site', '399'),
                exact: true
              },
              {
                path: '/ai-native-book/docs/tutorial-basics/markdown-features',
                component: ComponentCreator('/ai-native-book/docs/tutorial-basics/markdown-features', 'c98'),
                exact: true
              },
              {
                path: '/ai-native-book/docs/tutorial-extras/manage-docs-versions',
                component: ComponentCreator('/ai-native-book/docs/tutorial-extras/manage-docs-versions', '7c9'),
                exact: true
              },
              {
                path: '/ai-native-book/docs/tutorial-extras/translate-your-site',
                component: ComponentCreator('/ai-native-book/docs/tutorial-extras/translate-your-site', 'a44'),
                exact: true
              },
              {
                path: '/ai-native-book/docs/video_embedding_instructions',
                component: ComponentCreator('/ai-native-book/docs/video_embedding_instructions', '148'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/ai-native-book/docs/vision-language-action/',
                component: ComponentCreator('/ai-native-book/docs/vision-language-action/', '1b4'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/ai-native-book/docs/vision-language-action/capstone-autonomous-humanoid',
                component: ComponentCreator('/ai-native-book/docs/vision-language-action/capstone-autonomous-humanoid', '7b1'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/ai-native-book/docs/vision-language-action/llm-cognitive-planning',
                component: ComponentCreator('/ai-native-book/docs/vision-language-action/llm-cognitive-planning', '42f'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/ai-native-book/docs/vision-language-action/voice-to-action',
                component: ComponentCreator('/ai-native-book/docs/vision-language-action/voice-to-action', '78d'),
                exact: true,
                sidebar: "mainSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/ai-native-book/',
    component: ComponentCreator('/ai-native-book/', '54f'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
