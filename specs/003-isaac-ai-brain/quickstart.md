# Quickstart Guide: NVIDIA Isaac Robotics System

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS or Windows 10/11 with WSL2
- NVIDIA GPU with CUDA support (RTX 30xx/40xx series recommended)
- At least 16GB RAM
- 100GB free disk space for Isaac Sim
- ROS 2 Humble Hawksbill installed

### Software Dependencies
- NVIDIA Driver version 525 or higher
- CUDA Toolkit 12.x
- Isaac Sim (Omniverse-based)
- Isaac ROS packages
- Nav2 navigation stack
- Docker and Docker Compose

## Installation Steps

### 1. Install NVIDIA Drivers and CUDA
```bash
# Check GPU and install appropriate drivers
sudo apt update
sudo ubuntu-drivers autoinstall
sudo reboot

# Verify installation
nvidia-smi
nvcc --version
```

### 2. Install ROS 2 Humble
```bash
# Setup locale
sudo locale-gen en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions
```

### 3. Install Isaac ROS Packages
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Install Isaac ROS dependencies
sudo apt install python3-pip
pip3 install rosidl_parser

# Clone and build Isaac ROS packages
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git

# Build the workspace
cd ~/isaac_ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select isaac_ros_common
colcon build --symlink-install
```

### 4. Install Nav2
```bash
# Install Nav2 packages
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-nav2-gui-plugins ros-humble-nav2-rviz-plugins
```

### 5. Install Isaac Sim
1. Download Isaac Sim from NVIDIA Developer website
2. Extract and run the installer
3. Follow the installation wizard
4. Launch Isaac Sim and verify it runs properly

## Basic Usage Examples

### 1. Launch Isaac Sim with a Robot
```bash
# Navigate to Isaac Sim directory
cd ~/isaac-sim
./isaac-sim.launch.sh
```

### 2. Run Visual SLAM with Isaac ROS
```bash
# Source the workspace
source ~/isaac_ros_ws/install/setup.bash

# Launch Isaac Visual SLAM
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
```

### 3. Start Navigation with Nav2
```bash
# Source ROS 2 and the workspace
source /opt/ros/humble/setup.bash
source ~/isaac_ros_ws/install/setup.bash

# Launch navigation
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

## Troubleshooting

### Common Issues:
1. **CUDA Not Found**: Ensure NVIDIA drivers and CUDA toolkit are properly installed
2. **GPU Memory Issues**: Close other GPU-intensive applications
3. **ROS Package Not Found**: Source the ROS environment and rebuild packages
4. **Isaac Sim Crashes**: Check GPU driver compatibility and available VRAM

### Verification Commands:
```bash
# Check GPU availability
nvidia-smi

# Check ROS 2 installation
ros2 topic list

# Check Isaac ROS nodes
ros2 node list
```

## Next Steps
1. Follow the detailed chapters in this module to learn about each component
2. Experiment with sample robots and environments in Isaac Sim
3. Integrate perception and navigation pipelines
4. Test with real robot hardware when ready