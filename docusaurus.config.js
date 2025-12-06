// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const lightCodeTheme = require('prism-react-renderer').themes.github;
const darkCodeTheme = require('prism-react-renderer').themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Master Embodied Intelligence',
  favicon: 'img/favicon.ico',

  // Custom configuration for Chatbot
  customFields: {
    // This URL will be used by the chatbot to connect to the backend
    // In production (Vercel), this should be the deployed backend URL
    chatbotUrl: process.env.CHATBOT_URL || 'http://localhost:8000',
  },

  // GitHub Pages deployment configuration
  url: 'https://Zeenat-Somroo911.github.io',
  baseUrl: '/physical-ai-textbook/',

  // GitHub pages deployment config
  organizationName: 'Zeenat-Somroo911',
  projectName: 'physical-ai-textbook',

  // Deployment settings
  deploymentBranch: 'gh-pages',
  trailingSlash: false,

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

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
          editUrl: 'https://github.com/Zeenat-Somroo911/physical-ai-textbook/edit/main/',
        },
        blog: false,
        theme: {
          customCss: [
            require.resolve('./src/css/custom.css'),
            require.resolve('./src/css/rtl.css'),
          ],
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Dark mode by default
      colorMode: {
        defaultMode: 'dark',
        disableSwitch: false,
        respectPrefersColorScheme: false,
      },
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Physical AI Logo',
          src: '/img/logo.png',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Textbook',
          },
          {
            label: 'Module 1: ROS2',
            to: '/docs/module-01-ros2/intro',
            position: 'left',
          },
          {
            label: 'Module 2: Simulation',
            to: '/docs/module-02-simulation/intro',
            position: 'left',
          },
          {
            label: 'Module 3: Isaac',
            to: '/docs/module-03-isaac/intro',
            position: 'left',
          },
          {
            label: 'Module 4: VLA',
            to: '/docs/module-04-vla/intro',
            position: 'left',
          },
          {
            label: 'Projects',
            to: '/docs/projects/overview',
            position: 'left',
          },
          {
            href: 'https://github.com/Zeenat-Somroo911/physical-ai-textbook',
            label: 'GitHub',
            position: 'right',
            className: 'navbar-github-link',
          },

          {
            type: 'custom-urdu-button',
            position: 'right',
          },
          {
            type: 'custom-auth-button',
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
                label: 'Module 1: ROS2',
                to: '/docs/module-01-ros2/intro',
              },
              {
                label: 'Module 2: Simulation',
                to: '/docs/module-02-simulation/intro',
              },
              {
                label: 'Module 3: Isaac',
                to: '/docs/module-03-isaac/intro',
              },
              {
                label: 'Module 4: VLA',
                to: '/docs/module-04-vla/intro',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'Projects',
                to: '/docs/projects/overview',
              },
              {
                label: 'Code Examples',
                href: '/code-examples',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/Zeenat-Somroo911/physical-ai-textbook',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built with Docusaurus.`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
        additionalLanguages: ['python', 'bash', 'json', 'yaml', 'toml'],
      },
    }),
};

module.exports = config;

