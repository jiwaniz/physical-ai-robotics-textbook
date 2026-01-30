import React from 'react';
// Import the original mapper
import MDXComponents from '@theme-original/MDXComponents';
// Import custom components
import { UrduToggle } from '@site/src/components/UrduToggle';

export default {
  // Re-use the default mapping
  ...MDXComponents,
  // Add custom components
  UrduToggle,
};
