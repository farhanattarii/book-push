# Quickstart: Docusaurus UI Upgrade

## Overview
This quickstart guide will help you set up and customize the upgraded UI for the Docusaurus-based book_frontend project. The upgrade includes modern visual design, improved navigation, and responsive compatibility while preserving all existing content.

## Prerequisites
- Node.js 18+ installed
- npm or yarn package manager
- Git for version control
- Text editor or IDE

## Installation

### 1. Clone the Repository
```bash
git clone <repository-url>
cd ai-native-book-claude/book_frontend
```

### 2. Install Dependencies
```bash
npm install
# or
yarn install
```

### 3. Start Development Server
```bash
npm run start
# or
yarn start
```

The site will be available at `http://localhost:3000`.

## Customizing the UI

### 1. Theme Configuration
Update the theme configuration in `docusaurus.config.js`:

```javascript
module.exports = {
  // ... other config
  themeConfig: {
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
    navbar: {
      // Customize navigation bar appearance
    },
    footer: {
      // Customize footer appearance
    }
  }
};
```

### 2. Custom CSS
Add custom styles in `src/css/custom.css`:

```css
/* Custom color palette */
:root {
  --ifm-color-primary: #your-primary-color;
  --ifm-color-primary-dark: #your-dark-color;
  --ifm-color-primary-darker: #your-darker-color;
}

/* Custom typography */
html {
  font-family: 'Your-Font', sans-serif;
}

/* Responsive layout adjustments */
@media (max-width: 996px) {
  /* Mobile-specific styles */
}
```

### 3. Custom Components
Create custom React components in `src/components/`:

```jsx
// Example custom component
import React from 'react';
import styles from './CustomComponent.module.css';

function CustomComponent() {
  return (
    <div className={styles.container}>
      {/* Your custom UI component */}
    </div>
  );
}

export default CustomComponent;
```

## Building for Production
```bash
npm run build
# or
yarn build
```

The built site will be in the `build/` directory.

## Deployment
The site can be deployed to GitHub Pages, Netlify, Vercel, or any static hosting service:

```bash
npm run deploy
# or follow your hosting provider's deployment instructions
```

## Key Configuration Files
- `docusaurus.config.js` - Main site configuration
- `sidebars.js` - Navigation structure
- `src/css/custom.css` - Custom styling
- `src/components/` - Custom React components
- `static/` - Static assets (images, files)