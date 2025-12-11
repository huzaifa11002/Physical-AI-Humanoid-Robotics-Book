import { themes as prismThemes } from 'prism-react-renderer';
import type { Config } from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics Book',
  tagline: 'A comprehensive guide to building intelligent embodied systems.',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://physical-ai-humanoid-robotics-book-bice.vercel.app/', // Placeholder for GitHub Pages deployment
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/', // For GitHub Pages deployment, adjust as needed

  // GitHub pages deployment config.
  organizationName: 'specifykit', // Usually your GitHub org/user name.
  projectName: 'ai-native-book', // Usually your repo name.

  onBrokenLinks: 'throw',

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
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/huzaifa11002/Physical-AI-Humanoid-Robotics-Book/tree/main/', // Changed to project repo
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/huzaifa11002/Physical-AI-Humanoid-Robotics-Book/tree/main/', // Changed to project repo
          // Useful options to enforce blogging best practices
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI Book',
      logo: {
        alt: 'My Site Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'mainSidebar', // This will be changed by sidebars.ts later
          position: 'left',
          label: 'Docs', // Changed label to Docs
        },
        { to: '/blog', label: 'Blog', position: 'left' },
        {
          href: 'https://github.com/huzaifa11002/Physical-AI-Humanoid-Robotics-Book', // Changed to project repo
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
            { label: 'Robotic Nervous System', to: '/docs/robotic-nervous-system' },
            { label: 'Digital Twin', to: '/docs/digital-twin' },
            { label: 'AI Robot Brain', to: '/docs/ai-robot-brain' },
            { label: 'Vision Language Action', to: '/docs/vision-language-action' },
          ],
        },
        {
          title: 'Resources',
          items: [
            { label: 'GitHub Repository', href: 'https://github.com/huzaifa11002/Physical-AI-Humanoid-Robotics-Book' },
            { label: 'Instructor Guide', to: '/docs/instructor-guide' },
            { label: 'Glossary', to: '/docs/glossary' },
          ],
        },
        {
          title: 'Community',
          items: [
            { label: 'Discussions', href: 'https://github.com/huzaifa11002/Physical-AI-Humanoid-Robotics-Book/discussions' },
            { label: 'Issues', href: 'https://github.com/huzaifa11002/Physical-AI-Humanoid-Robotics-Book/issues' },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI Book. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
