# Quickstart Guide: Docusaurus UI Upgrade

## Overview
This guide provides the essential steps to begin implementing the UI upgrade for the Docusaurus-based book frontend. Follow these steps to set up your development environment and start customizing the UI.

## Prerequisites
- Node.js (v16 or higher)
- npm or yarn package manager
- Git
- A modern code editor (VS Code recommended)

## Setup Instructions

### 1. Clone and Navigate to the Project
```bash
cd book-frontend
```

### 2. Install Dependencies
```bash
# Using npm
npm install

# Or using yarn
yarn install
```

### 3. Start Development Server
```bash
# Using npm
npm run start

# Or using yarn
yarn start
```

This will start the Docusaurus development server at http://localhost:3000

## Key Files for UI Customization

### Theme Components
Override default Docusaurus components by creating files in `src/theme/`:
- `src/theme/Layout.js` - Main layout wrapper
- `src/theme/Navbar.js` - Top navigation bar
- `src/theme/DocSidebar.js` - Sidebar navigation
- `src/theme/MDXComponents.js` - Components for MDX content rendering

### CSS Files
Add custom styles in:
- `src/css/custom.css` - Main custom styles
- `src/theme/*` - Component-specific styles

### Configuration
Modify site configuration in:
- `docusaurus.config.ts` - Site-wide configuration including themes
- `sidebars.ts` - Navigation sidebar structure

## Implementation Steps

### Phase 1: Basic Styling
1. Create `src/css/custom.css` for global styles
2. Add CSS variables for consistent theming
3. Implement typography improvements
4. Add responsive breakpoints

### Phase 2: Component Overrides
1. Create custom theme components in `src/theme/`
2. Override Navbar for enhanced header
3. Customize DocSidebar for improved navigation
4. Implement theme switching functionality

### Phase 3: Advanced Features
1. Add light/dark theme toggle
2. Implement enhanced sidebar with search
3. Optimize for mobile responsiveness
4. Add accessibility enhancements

## Docusaurus Theme Override Pattern

To override a Docusaurus component, create a file in `src/theme/` with the same name:

```
src/
└── theme/
    ├── Navbar.js          # Overrides @theme/Navbar
    ├── DocSidebar.js      # Overrides @theme/DocSidebar
    └── Layout.js          # Overrides @theme/Layout
```

## Testing Your Changes

### Local Testing
```bash
# Start development server
npm run start

# Build for production
npm run build

# Serve production build locally
npm run serve
```

### Browser Testing
- Test in Chrome, Firefox, Safari, and Edge
- Use browser dev tools to test responsive layouts
- Verify keyboard navigation works properly
- Check contrast ratios for accessibility

## Common Customization Examples

### Custom CSS Variables
```css
:root {
  --ifm-color-primary: #your-primary-color;
  --ifm-font-family-base: 'Your-Font', system-ui, sans-serif;
  --ifm-spacing-horizontal: 1.5rem;
}
```

### Custom Navbar Component
```jsx
import React from 'react';
import Navbar from '@theme-original/Navbar';

export default function NavbarWrapper(props) {
  return (
    <>
      {/* Custom header content */}
      <Navbar {...props} />
    </>
  );
}
```

## Troubleshooting

### Component Override Not Working
- Ensure the filename matches the component being overridden
- Check that the file is in the correct `src/theme/` directory
- Restart the development server after creating new files

### Styles Not Applying
- Check CSS specificity - you may need more specific selectors
- Verify that custom CSS is imported in the right place
- Use browser dev tools to inspect element styles

## Next Steps
1. Review the [Data Model](./data-model.md) for detailed entity definitions
2. Check the [Implementation Plan](./plan.md) for complete architecture
3. Follow the [Tasks](../tasks.md) for detailed implementation steps