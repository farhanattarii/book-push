# Physics Simulation with Gazebo

This section covers setting up and configuring physics-based simulation environments using Gazebo for humanoid robot models. Physics simulation is the foundation for testing robot control algorithms, understanding robot dynamics, and validating behaviors before deploying to real hardware.

## Learning Objectives

After completing this section, you will be able to:
- Install and configure Gazebo for humanoid robot simulation
- Import and configure robot models in Gazebo
- Set up physics properties including gravity, friction, and collision detection
- Control robot joints and observe physics responses
- Create custom simulation environments with obstacles

## Installing Gazebo

### Option 1: Install Gazebo Garden (Recommended)

Gazebo Garden is the latest stable version with good ROS 2 integration:

```bash
# Add the OSRF APT repository
sudo apt update && sudo apt install wget
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'

# Download and install the Gazebo signing key
wget https://packages.osrfoundation.org/gazebo.gpg -O - | sudo gpg --dearmor -o /usr/share/keyrings/gazebo-archive.keyring

# Update the APT cache
sudo apt update

# Install Gazebo Garden
sudo apt install gz-garden
```

### Option 2: Install through ROS 2

If you have ROS 2 installed, you can install the Gazebo ROS packages:

```bash
sudo apt update
sudo apt install ros-humble-gazebo-*
```

## Verifying Installation

Test your Gazebo installation:

```bash
gz sim
```

If Gazebo starts successfully, you should see the Gazebo interface with a default empty world.

## Basic Gazebo Concepts

### Worlds
A world file defines the environment for your simulation, including:
- Physical properties (gravity, magnetic field)
- Models and their initial positions
- Lighting and visual effects
- Plugins and sensors

### Models
Models represent objects in the simulation, including:
- Robot models (URDF/SDF format)
- Environment objects (tables, walls, obstacles)
- Sensors and actuators

### Plugins
Plugins extend Gazebo's functionality:
- Physics engine plugins
- Sensor plugins
- Controller plugins
- GUI plugins

## Creating Your First Simulation

Let's create a simple world file to familiarize ourselves with Gazebo:

1. Create a directory for your simulation files:
   ```bash
   mkdir -p ~/gazebo_simulations/first_world
   cd ~/gazebo_simulations/first_world
   ```

2. Create a basic world file named `simple_world.sdf`:
   ```xml
   <?xml version="1.0" ?>
   <sdf version="1.7">
     <world name="simple_world">
       <!-- Include a ground plane -->
       <include>
         <uri>model://ground_plane</uri>
       </include>

       <!-- Include a sun for lighting -->
       <include>
         <uri>model://sun</uri>
       </include>

       <!-- Add a simple box model -->
       <model name="box">
         <pose>0 0 0.5 0 0 0</pose>
         <link name="link">
           <collision name="collision">
             <geometry>
               <box>
                 <size>1 1 1</size>
               </box>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <box>
                 <size>1 1 1</size>
               </box>
             </geometry>
           </visual>
         </link>
       </model>
     </world>
   </sdf>
   ```

3. Run Gazebo with your world:
   ```bash
   gz sim -r simple_world.sdf
   ```

## Integrating with ROS 2

To use Gazebo with ROS 2, you'll need the Gazebo ROS packages:

```bash
# Install Gazebo ROS packages
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-ros-gz

# Source your ROS 2 installation
source /opt/ros/humble/setup.bash
```

### Launching Gazebo with ROS 2

You can launch Gazebo with ROS 2 integration using:

```bash
# Launch Gazebo server (headless)
ros2 run gazebo_ros gazebo

# Or launch with a specific world
ros2 launch gazebo_ros empty_world.launch.py world_name:=path_to_your_world.sdf
```

### Robot Models in Gazebo

#### Importing URDF Models

To import a URDF robot model into Gazebo:

1. Convert your URDF to SDF format (optional but recommended):
   ```bash
   # This is done automatically when you spawn the URDF in Gazebo
   ```

2. Launch Gazebo with your robot:
   ```bash
   # Using ROS 2 launch file
   ros2 launch your_robot_description spawn_robot.launch.py
   ```

#### Physics Properties Configuration

Key physics properties you can configure:

- **Gravity**: Default is -9.81 m/s² in the z direction
- **Friction**: Defines how objects interact when in contact
- **Damping**: Simulates energy loss in joints and motion
- **Contact properties**: How objects respond to collisions

## Common Issues and Troubleshooting

### Issue: Gazebo crashes or fails to start
- **Solution**: Check if you have proper graphics drivers installed
- Try running with software rendering: `export LIBGL_ALWAYS_SOFTWARE=1`

### Issue: Slow performance
- **Solution**: Reduce physics update rate in world file
- Check system resources (CPU, RAM usage)

### Issue: ROS 2 plugins not loading
- **Solution**: Verify Gazebo and ROS 2 versions are compatible
- Check that `gazebo_ros_pkgs` are properly installed