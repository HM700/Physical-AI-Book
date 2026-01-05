// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Chapters',
      items: ['chapters/chapter-1/lesson-1', 'chapters/chapter-1/lesson-2', 'chapters/chapter-1/lesson-3'],
    },
    {
      type: 'category',
      label: 'Reference',
      items: ['reference/glossary', 'reference/quick-reference'],
    },
    {
      type: 'category',
      label: 'Examples',
      items: ['examples/index'],
    },
  ],
};

module.exports = sidebars;