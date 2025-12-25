# Data Model: Digital Twin Module

## Core Entities

### 1. Module Structure
- **Name**: Digital Twin Module (Physics Simulation & 3D Visualization)
- **Description**: Educational module for AI and robotics students
- **Chapters**: [Chapter 1: Physics Simulation, Chapter 2: Digital Twins & HRI, Chapter 3: Sensor Simulation]
- **Format**: Markdown documentation with Docusaurus integration
- **Target Audience**: AI and robotics students

### 2. Chapter Entity
- **ID**: Unique identifier (e.g., "chapter-1-physics-simulation")
- **Title**: Display title for the chapter
- **Sections**: Array of section entities
- **Learning Objectives**: Array of educational goals
- **Prerequisites**: Array of required knowledge/skills
- **Hands-on Exercises**: Array of practical exercises
- **Duration**: Estimated time to complete

### 3. Section Entity
- **ID**: Unique identifier within chapter
- **Title**: Display title for the section
- **Content**: Markdown content
- **Type**: [Theory, Tutorial, Exercise, Reference]
- **Dependencies**: Array of prerequisite sections
- **Assets**: Array of related files (images, code samples, etc.)

### 4. Exercise Entity
- **ID**: Unique identifier within chapter
- **Title**: Exercise title
- **Description**: Brief overview of the exercise
- **Instructions**: Step-by-step guidance
- **Expected Outcome**: What students should achieve
- **Difficulty**: [Beginner, Intermediate, Advanced]
- **Estimated Time**: Time needed to complete
- **Required Tools**: Software/hardware needed

### 5. Asset Entity
- **ID**: Unique identifier
- **Type**: [Image, Video, Code, Configuration, Model]
- **Path**: Relative path from documentation root
- **Description**: Brief description of the asset
- **Usage Context**: Where and how the asset is used
- **Format**: File format (e.g., ".png", ".py", ".urdf")

### 6. Configuration Entity
- **ID**: Unique identifier for configuration
- **Type**: [Gazebo, Unity, Sensor, Environment]
- **Format**: Configuration format (e.g., URDF, SDF, YAML)
- **Content**: Configuration data
- **Description**: Purpose and usage of the configuration
- **Parameters**: Key-value pairs of configurable options

### 7. Simulation Environment Entity
- **ID**: Unique identifier
- **Name**: Display name for the environment
- **Description**: Overview of the environment
- **Terrain Types**: Array of supported terrain types
- **Obstacles**: Array of object types that can be placed
- **Physics Properties**: Gravity, friction, etc.
- **Supported Robots**: List of compatible robot models

### 8. Sensor Data Entity
- **ID**: Unique identifier
- **Type**: [LiDAR, Depth Camera, IMU, Camera, GPS]
- **Format**: Data format (e.g., PointCloud2, Image, Imu)
- **Parameters**: Range, resolution, noise characteristics
- **Output**: Expected data structure
- **Validation Criteria**: How to verify data quality

## Relationships

1. **Module** contains many **Chapters**
2. **Chapter** contains many **Sections** and **Exercises**
3. **Section** uses many **Assets**
4. **Exercise** may reference many **Assets** and **Configurations**
5. **Simulation Environment** supports many **Configurations**
6. **Robot Model** can have many **Sensor Data** streams
7. **Configuration** can be used by many **Exercises**

## Validation Rules

1. Each **Chapter** must have at least one **Section**
2. Each **Exercise** must have a defined **Difficulty** level
3. Each **Asset** must have a valid **Path** within the documentation structure
4. Each **Configuration** must specify a valid **Format**
5. **Learning Objectives** must align with the **Chapter** content
6. **Prerequisites** must not create circular dependencies
7. **Estimated Times** must be positive values