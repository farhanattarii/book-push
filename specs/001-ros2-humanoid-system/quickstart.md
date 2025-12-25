# Quickstart: ROS 2 for Humanoid Robotics Module

## Prerequisites

- Node.js (v18 or higher)
- npm or yarn package manager
- Git for version control
- Basic command line knowledge

## Setup Instructions

### 1. Clone or Initialize the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Install Dependencies
```bash
npm install
# or
yarn install
```

### 3. Initialize Docusaurus Project (if not already done)
```bash
npm init docusaurus@latest website classic
# Follow prompts to create the Docusaurus site
```

### 4. Start Development Server
```bash
npm run start
# or
yarn start
```

This will start a local development server at `http://localhost:3000` with hot reloading enabled.

## Adding the ROS 2 Module Content

### 1. Create the Module Directory
```bash
mkdir -p docs/ros2-humanoid
```

### 2. Create the Three Chapters

Create the following files in the `docs/ros2-humanoid/` directory:

- `intro-to-ros2.md` - Introduction to ROS 2 for Physical AI
- `communication-model.md` - ROS 2 Communication Model
- `robot-structure-urdf.md` - Robot Structure with URDF

### 3. Update Sidebar Configuration

In `sidebar.js` or `sidebars.js`, add the new module:

```javascript
module.exports = {
  docs: [
    // ... existing sidebar items
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'ros2-humanoid/intro-to-ros2',
        'ros2-humanoid/communication-model',
        'ros2-humanoid/robot-structure-urdf'
      ],
    },
  ],
};
```

### 4. Run and Verify

After adding the content and updating the sidebar, restart the development server:

```bash
npm run start
```

Navigate to `http://localhost:3000` and verify that the new module appears in the sidebar and all chapters are accessible.

## Building for Production

To build the static site for deployment:

```bash
npm run build
# or
yarn build
```

The built site will be available in the `build/` directory and can be deployed to GitHub Pages or any static hosting service.