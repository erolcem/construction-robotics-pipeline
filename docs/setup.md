# Universal Robots ROS 2 Project Setup Guide

This guide will help you set up the Universal Robots ROS 2 control and simulation environment.

## Prerequisites

### Host System Requirements
- Ubuntu 22.04 LTS
- Docker and Docker Compose
- Git
- At least 8GB RAM recommended
- Graphics card with OpenGL support (for visualization)

### Software Dependencies
```bash
# Install Docker
sudo apt-get update
sudo apt-get install docker.io docker-compose
sudo usermod -aG docker $USER
# Log out and back in for group changes to take effect

# Install Git (if not already installed)
sudo apt-get install git

# For GUI applications in Docker
sudo apt-get install x11-xserver-utils
```

## Quick Start

### 1. Clone the Repository
```bash
git clone <your-repo-url>
cd fyp_v2
```

### 2. Build Docker Environment
```bash
# Build the Docker image (this will take several minutes on first run)
docker-compose build

# Verify the build
docker-compose config
```

### 3. Start Simulation
```bash
# Option 1: Using the convenience script
./scripts/run_simulation.sh --docker

# Option 2: Using Docker Compose directly
docker-compose up ur-simulation
```

### 4. Verify Installation
In a new terminal, connect to the running container:
```bash
docker exec -it ur_ros2_simulation bash

# Inside the container, check if everything is working
ros2 pkg list | grep ur_
ros2 topic list
```

## Development Setup

### For Host Development (Optional)
If you want to develop directly on the host system:

```bash
# Install ROS 2 Humble
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop-full

# Install additional packages
sudo apt install python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update

# Run the setup script
./scripts/setup_environment.sh
```

## Configuration

### Robot Configuration
Edit `config/robot_config.yaml` to configure:
- Robot model (ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e, ur20)
- Joint limits and safety parameters
- Network settings for real robot connection
- Control parameters

### Simulation Configuration
Edit `config/simulation_config.yaml` to configure:
- Simulation environment
- Sensor parameters
- Visualization settings
- Performance options

## Usage Examples

### Simulation Mode
```bash
# Start basic simulation
ros2 launch launch/ur_sim_launch.py

# Start with specific robot model
ros2 launch launch/ur_sim_launch.py robot_model:=ur10e

# Start without GUI (headless)
ros2 launch launch/ur_sim_launch.py use_gui:=false
```

### Real Robot Mode
```bash
# Connect to real robot (replace with actual IP)
ros2 launch launch/ur_real_launch.py robot_ip:=192.168.1.100

# With specific robot model
ros2 launch launch/ur_real_launch.py robot_ip:=192.168.1.100 robot_model:=ur5e
```

### State Monitoring
```bash
# Start state monitoring
ros2 run ur_state_monitor state_monitor.py

# View robot state in real-time
ros2 topic echo /robot_state_summary
```

## Troubleshooting

### Docker Issues
```bash
# If X11 forwarding doesn't work
xhost +local:
export DISPLAY=:0

# Check Docker permissions
sudo usermod -aG docker $USER
# Then log out and back in

# Reset Docker if needed
docker system prune -a
```

### ROS 2 Issues
```bash
# Check ROS 2 environment
printenv | grep ROS

# Verify nodes are running
ros2 node list

# Check for error messages
ros2 node info /robot_controller
```

### Network Issues (Real Robot)
```bash
# Test network connectivity
ping <robot_ip>

# Check robot dashboard
curl http://<robot_ip>:29999

# Verify robot is in remote control mode
# Use UR teach pendant to enable external control
```

## Next Steps

After successful setup, you can:
1. Test basic robot movements
2. Explore the control interface
3. Add custom control logic
4. Integrate additional sensors
5. Extend to multi-robot systems

## Support

If you encounter issues:
1. Check the logs in `/workspace/logs/`
2. Verify all prerequisites are installed
3. Ensure proper network configuration for real robots
4. Review the troubleshooting section above
