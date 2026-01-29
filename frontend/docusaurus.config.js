// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',

  // Custom fields accessible via useDocusaurusContext
  customFields: {
    // Production API URL (Hugging Face Spaces)
    apiUrlProd: 'https://jiwaniz-physical-ai-backend.hf.space',
    // Development API URL (local)
    apiUrlDev: 'http://localhost:8001',
    // Supabase configuration
    supabaseUrl: 'https://vaumzrgjpnkahbqedcgp.supabase.co',
    supabaseAnonKey: 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6InZhdW16cmdqcG5rYWhicWVkY2dwIiwicm9sZSI6ImFub24iLCJpYXQiOjE3Njk0MDE1NTAsImV4cCI6MjA4NDk3NzU1MH0.yxV9fvKagF-gFzoZjqGDoyojcv0e2QC3HEPg9zUwgOs',
  },
  tagline: 'Interactive textbook with AI-powered learning assistance',
  favicon: 'img/favicon.svg',

  // Set the production url of your site here
  url: 'https://jiwaniz.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/physical-ai-robotics-textbook/',

  // GitHub pages deployment config
  organizationName: 'jiwaniz', // Usually your GitHub org/user name
  projectName: 'physical-ai-robotics-textbook', // Usually your repo name

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang
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
          routeBasePath: '/', // Serve docs at the site root
          editUrl: 'https://github.com/jiwaniz/physical-ai-robotics-textbook/tree/001-rag-textbook-platform/frontend/',
        },
        blog: false,
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themes: [
    [
      require.resolve('@easyops-cn/docusaurus-search-local'),
      {
        hashed: true,
        language: ['en'],
        highlightSearchTermsOnTargetPage: true,
        explicitSearchResultPath: true,
        docsRouteBasePath: '/',
        indexBlog: false,
        indexPages: false,
        indexDocs: true,
      },
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      navbar: {
        title: 'Physical AI & Robotics',
        logo: {
          alt: 'Site Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'doc',
            docId: 'intro',
            position: 'left',
            label: 'Course',
          },
          {
            href: 'https://github.com/jiwaniz/physical-ai-robotics-textbook',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Course',
            items: [
              {
                label: 'Getting Started',
                to: '/intro',
              },
              {
                label: 'Assessments',
                to: '/assessments',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'ROS 2 Documentation',
                href: 'https://docs.ros.org/en/humble/',
              },
              {
                label: 'NVIDIA Isaac Sim',
                href: 'https://docs.omniverse.nvidia.com/isaacsim/latest/',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/jiwaniz/physical-ai-robotics-textbook',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Course. Built with Docusaurus.`,
      },
      prism: {
        theme: require('prism-react-renderer').themes.github,
        darkTheme: require('prism-react-renderer').themes.dracula,
        additionalLanguages: ['bash', 'python', 'cpp', 'cmake', 'yaml'],
      },
      colorMode: {
        defaultMode: 'light',
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },
      docs: {
        sidebar: {
          hideable: true,
          autoCollapseCategories: true,
        },
      },
      tableOfContents: {
        minHeadingLevel: 2,
        maxHeadingLevel: 4,
      },
    }),
};

module.exports = config;
