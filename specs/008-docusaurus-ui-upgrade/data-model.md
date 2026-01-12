# Data Model: Docusaurus UI Upgrade

## Overview
This document describes the data models and entities relevant to the Docusaurus UI upgrade project. Since this is primarily a UI/UX enhancement project with no changes to content structure, the data model focuses on the presentation layer and configuration entities.

## Key Entities

### Page Layout
- **Definition**: The structural organization of content on individual pages including headers, navigation, main content, and footers
- **Attributes**:
  - headerHeight: number (in pixels or rem)
  - sidebarWidth: number (in pixels or rem)
  - contentWidth: number (in pixels or rem)
  - footerHeight: number (in pixels or rem)
  - breakpoints: object (mobile, tablet, desktop dimensions)
- **Relationships**: Contains Navigation System, Theme Configuration, Responsive Components

### Navigation System
- **Definition**: The sidebar, top navigation, and internal linking system that enables users to move through the book content
- **Attributes**:
  - sidebarStructure: array (hierarchical organization of content)
  - navigationDepth: number (max depth of navigation tree)
  - searchEnabled: boolean (whether search functionality is available)
  - breadcrumbsEnabled: boolean (whether breadcrumbs are displayed)
  - previousNextLinks: boolean (whether previous/next page links are shown)
- **Relationships**: Connected to Page Layout, Theme Configuration

### Theme Configuration
- **Definition**: The styling settings that control visual appearance including colors, fonts, spacing, and responsive behavior
- **Attributes**:
  - primaryColor: string (hex code or CSS variable)
  - secondaryColor: string (hex code or CSS variable)
  - fontFamily: string (font family name)
  - fontSize: object (base, small, medium, large sizes)
  - spacing: object (spacing scale for margins, padding)
  - lightMode: object (color scheme for light theme)
  - darkMode: object (color scheme for dark theme)
  - borderRadius: number (for UI elements)
  - shadow: object (elevation shadows)
- **Relationships**: Applied to Page Layout, Navigation System, Responsive Components

### Responsive Components
- **Definition**: UI elements that adapt their presentation based on screen size and device characteristics
- **Attributes**:
  - breakpoints: object (mobile, tablet, desktop thresholds)
  - mobileLayout: object (layout configuration for mobile)
  - desktopLayout: object (layout configuration for desktop)
  - touchTargets: object (minimum touch target sizes for mobile)
  - responsiveTypography: object (font scaling rules)
- **Relationships**: Influenced by Page Layout, Theme Configuration

## UI Configuration Schema

### Theme Schema
```json
{
  "theme": {
    "colors": {
      "primary": "#3178C6",
      "secondary": "#654FF0",
      "background": "#FFFFFF",
      "text": "#242526",
      "textSecondary": "#525354",
      "border": "#E1E1E1",
      "sidebarBackground": "#F8F9FA"
    },
    "typography": {
      "fontFamily": "'Inter', system-ui, sans-serif",
      "fontSize": {
        "base": "16px",
        "small": "0.875rem",
        "medium": "1rem",
        "large": "1.125rem",
        "xl": "1.25rem",
        "xxl": "1.5rem"
      }
    },
    "spacing": {
      "unit": "4px",
      "scale": [0, 4, 8, 16, 24, 32, 48, 64]
    },
    "breakpoints": {
      "mobile": "768px",
      "tablet": "992px",
      "desktop": "1200px"
    }
  }
}
```

### Layout Schema
```json
{
  "layout": {
    "sidebar": {
      "width": "300px",
      "mobileOpen": false,
      "position": "left"
    },
    "header": {
      "height": "60px",
      "sticky": true
    },
    "footer": {
      "height": "80px"
    },
    "content": {
      "maxWidth": "1200px",
      "padding": {
        "mobile": "16px",
        "desktop": "32px"
      }
    }
  }
}
```

## State Management

### Theme State
- **currentState**: light | dark | system
- **availableThemes**: array of available theme options
- **userPreference**: stored user preference for theme selection

### Navigation State
- **currentPath**: current page URL
- **expandedSections**: array of expanded sidebar sections
- **searchQuery**: current search input
- **activeBreadcrumb**: current position in content hierarchy

## Validation Rules
- All color values must meet WCAG 2.1 AA contrast requirements
- Font sizes must be responsive and accessible (minimum 16px equivalent)
- Touch targets must be at least 44px in mobile view
- All UI elements must be keyboard navigable
- Theme configurations must support both light and dark modes