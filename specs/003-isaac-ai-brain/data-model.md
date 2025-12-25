# Data Model: NVIDIA Isaac Robotics System

## Entities

### RobotConfiguration
- **id**: string (unique identifier)
- **name**: string (human-readable name)
- **type**: string (e.g., "humanoid", "wheeled", "quadrotor")
- **sensors**: array of Sensor objects
- **capabilities**: array of string (e.g., "navigation", "manipulation", "perception")
- **description**: string (detailed description)

### Sensor
- **id**: string (unique identifier)
- **type**: string (e.g., "camera", "lidar", "imu", "depth_sensor")
- **position**: Position object (x, y, z coordinates)
- **orientation**: Orientation object (quaternion)
- **parameters**: object (sensor-specific parameters)

### Position
- **x**: float (meters)
- **y**: float (meters)
- **z**: float (meters)

### Orientation
- **qx**: float (quaternion x)
- **qy**: float (quaternion y)
- **qz**: float (quaternion z)
- **qw**: float (quaternion w)

### SimulationEnvironment
- **id**: string (unique identifier)
- **name**: string (environment name)
- **description**: string (description of the environment)
- **scene_file**: string (path to USD file)
- **lighting_conditions**: string (e.g., "indoor", "outdoor", "dusk")
- **physics_properties**: PhysicsProperties object

### PhysicsProperties
- **gravity**: float (m/s^2)
- **friction**: float
- **bounce**: float

### NavigationGoal
- **id**: string (unique identifier)
- **pose**: Pose object (position and orientation)
- **priority**: integer (0-10 scale)
- **timeout**: integer (seconds)
- **constraints**: array of Constraint objects

### Pose
- **position**: Position object
- **orientation**: Orientation object

### Constraint
- **type**: string (e.g., "max_velocity", "forbidden_area", "preferred_path")
- **value**: object (constraint-specific value)
- **description**: string (explanation of constraint)

### NavigationResult
- **id**: string (unique identifier)
- **goal_id**: string (reference to NavigationGoal)
- **status**: string (e.g., "success", "failed", "timeout", "cancelled")
- **path_taken**: array of Pose objects
- **execution_time**: float (seconds)
- **distance_traveled**: float (meters)
- **energy_consumed**: float (Joules)

## Relationships
- RobotConfiguration has many Sensor
- SimulationEnvironment contains many RobotConfiguration
- NavigationGoal belongs to a RobotConfiguration
- NavigationResult belongs to a NavigationGoal

## State Transitions

### Navigation Goal States
- **PENDING** → Goal created but not yet executed
- **ACTIVE** → Goal execution in progress
- **SUCCESS** → Goal reached successfully
- **FAILED** → Goal execution failed
- **CANCELLED** → Goal cancelled by user/system
- **TIMEOUT** → Goal execution exceeded time limit

### Robot States
- **IDLE** → Robot is stationary and waiting for commands
- **NAVIGATING** → Robot is executing navigation task
- **PAUSED** → Navigation temporarily paused
- **EMERGENCY_STOP** → Robot stopped due to safety concerns
- **CHARGING** → Robot is charging its power source