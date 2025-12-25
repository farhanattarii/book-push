# Quickstart Guide: Digital Twin Module

## Prerequisites

Before starting with the Digital Twin Module, ensure you have the following installed:

1. **Node.js** (version 18 or higher)
2. **npm** or **yarn** package manager
3. **Git** for version control
4. **Python 3.x** (for potential ROS/Gazebo integration examples)

## Setup Instructions

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Install Dependencies

```bash
cd book_frontend
npm install
# or
yarn install
```

### 3. Start the Development Server

```bash
npm run start
# or
yarn start
```

This will start the Docusaurus development server at `http://localhost:3000`.

### 4. Navigate to the Digital Twin Module

Once the server is running, you can access the Digital Twin Module at:
`http://localhost:3000/docs/module-2`

## Module Structure Overview

The Digital Twin Module is organized into three main chapters:

### Chapter 1: Physics Simulation
- Setting up Gazebo simulation environments
- Importing and configuring robot models
- Understanding physics properties and constraints
- Hands-on exercises with basic robot control

### Chapter 2: Digital Twins & HRI
- Setting up Unity for visualization
- Creating high-fidelity 3D representations
- Human-robot interaction studies
- Synchronization between physics and visualization

### Chapter 3: Sensor Simulation & Validation
- LiDAR simulation techniques and parameters
- Depth camera and IMU simulation
- Validation methods for sensor data
- Practical exercises with sensor fusion

## First Steps

1. Start with Chapter 1 to understand the physics simulation fundamentals
2. Follow the hands-on exercises in each section
3. Progress through each chapter sequentially for optimal learning
4. Complete the exercises to reinforce your understanding

## Common Issues and Solutions

### Issue: Development server not starting
**Solution**: Ensure Node.js and npm are properly installed and in your PATH

### Issue: Missing documentation pages
**Solution**: Check that all required Markdown files are present in the `docs/module-2/` directory

### Issue: Links not working properly
**Solution**: Verify that all internal links follow Docusaurus conventions and use relative paths

## Next Steps

After completing the quickstart:
1. Review Chapter 1 content on physics simulation
2. Set up your local Gazebo environment following the documentation
3. Begin with the first hands-on exercise
4. Join the community forum for additional support and discussions