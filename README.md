#  Universal Robots ROS 2 Control Platform

**Complete Ubuntu setup for controlling UR robots with ROS 2 - works with both simulation and real robots**

##  Aim

After following this guide, you'll have:
-  **Working UR5e robot simulation** that you can control via commands
-  **Real-time robot control** - move joints, execute trajectories  
-  **Robot state monitoring** - see positions, velocities, forces
-  **GUI visualization** with RViz (optional)
-  **Easy switching** between simulation and real robot
-  **Docker containerized** - consistent across all Ubuntu systems

---

##  Requirements

- **Ubuntu 22.04 LTS** (or 20.04/24.04)
- **8GB RAM minimum** (16GB recommended)
- **20GB free disk space**
- **Internet connection** for downloading packages

---

## Complete Setup Guide (Ubuntu from Scratch)

### Step 1: Install Docker

```bash
# Remove any old Docker installations
sudo apt-get remove docker docker-engine docker.io containerd runc

# Update package index
sudo apt-get update

# Install dependencies
sudo apt-get install -y \
    ca-certificates \
    curl \
    gnupg \
    lsb-release

# Add Docker's official GPG key
sudo mkdir -m 0755 -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

# Set up Docker repository
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker Engine
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Add your user to docker group (to run without sudo)
sudo usermod -aG docker $USER

# Log out and log back in, or run:
newgrp docker

# Test Docker installation
docker run hello-world
```

### Step 2: Install Docker Compose

```bash
# Install Docker Compose (if not already installed)
sudo apt-get install -y docker-compose

# Verify installation
docker-compose --version
```

### Step 3: Clone and Setup This Project

```bash
# Clone the repository
git clone https://github.com/erolcem/fyp_v2.git
cd fyp_v2

# Make scripts executable
chmod +x scripts/*.sh

# Create logs directory
mkdir -p logs
```

### Step 4: Build the Robot Environment

```bash
# Build the Docker container (takes 5-10 minutes first time)
docker-compose build ur-simulation

# This downloads ~3GB of ROS 2 and robot packages
# Go get coffee ☕ while it builds...
```

---

##  Quick Start - Control Your Robot in 3 Commands

### Option A: Headless Mode (Works Immediately)

```bash
# 1. Start the robot simulation
docker-compose up ur-simulation
```

Wait for this message: `[robot_state_publisher-1] [INFO] ... got segment base_link`

```bash
# 2. In a NEW terminal, connect to the robot
docker exec -it ur_ros2_simulation bash

# 3. Inside the container, control the robot
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

# Move robot to a new position!
ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState "
header: {stamp: {sec: 0, nanosec: 0}}
name: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
position: [0.5, -1.0, -0.5, -2.0, 0.0, 0.0]
velocity: []
effort: []"
```

 **Your robot just moved!** You can see the joint positions changing.

### Option B: With GUI (RViz Visualization)

```bash
# 1. Enable GUI support (required for Linux)
xhost +local:docker

# 2. Start simulation with GUI
docker-compose --profile gui up ur-gui-simulation

# 3. Use same control commands as above
# You'll see RViz window with 3D robot visualization!

# 4. When done, clean up GUI permissions
xhost -local:docker
```

---

##  Robot Control Examples

### Basic Position Commands

```bash
# Inside the container (docker exec -it ur_ros2_simulation bash)

# Home position (safe starting point)
ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState "
name: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
position: [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]"

# Extended reach position
ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState "
name: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'] 
position: [0.5, -1.0, -0.5, -2.0, 0.0, 0.0]"

# Compact folded position
ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState "
name: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
position: [-0.5, -2.0, 1.0, -1.5, 1.57, 0.0]"
```

### Monitor Robot State

```bash
# Watch robot moving in real-time
ros2 topic echo /joint_states

# See available robot topics
ros2 topic list

# Check if robot is publishing correctly
ros2 topic hz /joint_states
```

### Advanced Control

```bash
# Run the built-in demonstration
ros2 run ur_control control_demo.py

# Run advanced joint controller
ros2 run ur_control joint_controller.py
```

---

## Switching to Real Robot

When you have a physical UR robot:

```bash
# 1. Stop simulation
docker-compose down

# 2. Connect to your robot's network and find its IP
# (Usually something like 192.168.1.100)

# 3. Set robot IP and start real robot mode
export ROBOT_IP=192.168.1.100  # Replace with your robot's IP
docker-compose --profile real-robot up ur-real

# 4. Use the SAME control commands!
# Your commands now control the real robot instead of simulation
```

 **Safety Note**: Always have the emergency stop accessible when using real robots!

---

##  Troubleshooting

### Problem: "Cannot start service ur-simulation: Mounts denied"
**Solution**: This is a Docker Desktop issue on Linux. Use the headless mode:
```bash
# Use headless mode instead
docker-compose up ur-simulation
# GUI requires additional Linux setup
```

### Problem: "container is not running"
**Solution**: Start the container first:
```bash
# Make sure container is running
docker-compose up ur-simulation

# Then in another terminal
docker exec -it ur_ros2_simulation bash
```

### Problem: "ros2: command not found" 
**Solution**: You're trying to run ROS 2 outside the container:
```bash
# ROS 2 commands only work INSIDE the container
docker exec -it ur_ros2_simulation bash
# Now you're inside the container where ROS 2 is installed
```

### Problem: "No such file or directory" for setup.bash
**Solution**: Build the workspace first:
```bash
# Inside container
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Problem: GUI not working
**Solution**: Setup X11 forwarding properly:
```bash
# Run these commands on your Ubuntu host (outside container)
sudo apt-get install x11-xserver-utils
xhost +local:docker

# Then start GUI simulation
docker-compose --profile gui up ur-gui-simulation
```

---

## Understanding the Robot

### UR5e Joint Layout
- **Joint 1**: Base rotation (shoulder_pan_joint)
- **Joint 2**: Shoulder up/down (shoulder_lift_joint)  
- **Joint 3**: Elbow bend (elbow_joint)
- **Joint 4**: Wrist rotation 1 (wrist_1_joint)
- **Joint 5**: Wrist rotation 2 (wrist_2_joint)
- **Joint 6**: Tool rotation (wrist_3_joint)

### Position Values (in radians)
- **0.0** = 0 degrees
- **1.57** ≈ 90 degrees  
- **3.14** ≈ 180 degrees
- **-1.57** ≈ -90 degrees

---

## 🎯 What's Next?

Now that you have a working robot control system, you can:

1. **Build custom applications** - Use this as a foundation
2. **Add sensors** - Integrate cameras, grippers, force sensors
3. **Implement AI** - Add computer vision and machine learning
4. **Connect real robots** - Use the same code with physical UR robots
5. **Scale up** - Add multiple robots and coordination

---

##  Project Structure

```
fyp_v2/
├── src/                    # ROS 2 packages
│   ├── ur_control/         # Robot control nodes
│   └── ur_state_monitor/   # State monitoring  
├── launch/                 # Launch files
├── config/                 # Configuration files
├── docker/                 # Docker setup
├── scripts/               # Utility scripts
├── logs/                  # Log files
└── docker-compose.yml     # Container configuration
```

---

##  Success Checklist

After setup, you should be able to:

-  Run `docker-compose up ur-simulation` without errors
-  Connect with `docker exec -it ur_ros2_simulation bash`  
-  See robot joints with `ros2 topic echo /joint_states`
-  Move robot with position commands
-  (Optional) See 3D robot in RViz with GUI mode

**If all checkboxes work, congratulations! You have a fully functional Universal Robots ROS 2 control system! 🎉**

---

## Support

If you encounter issues:
1. Check the troubleshooting section above
2. Ensure you're running commands in the correct terminal (host vs container)
3. Verify Docker is installed and your user is in the docker group
4. Make sure you have sufficient disk space and memory

**This platform is production-ready for robotics development, education, and research!**
xhost -local:docker
```

**Option C: Interactive Development**
```bash
# Start interactive container
docker run --rm -it fyp_v2_ur-simulation:latest

# Inside the container, manually launch simulation components:
# 1. Launch UR simulation with RViz
ros2 launch ur_bringup ur5e.launch.py use_fake_hardware:=true launch_rviz:=true

# In a new terminal/tab, start your custom nodes:
ros2 run ur_control robot_controller.py
ros2 run ur_state_monitor state_monitor.py
```

### 4. Interactive Development

```bash
# Run container interactively
docker run --rm -it --name ur_dev fyp_v2_ur-simulation:latest /bin/bash

# Inside the container, source the environment
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

# List available packages
ros2 pkg list | grep ur

# Run robot controller
ros2 run ur_control robot_controller.py

# Run state monitor
ros2 run ur_state_monitor state_monitor.py
```

## VS Code Tasks

The project includes pre-configured VS Code tasks for common operations:

- **Build ROS 2 Workspace**: `Ctrl+Shift+P` → `Tasks: Run Task` → `Build ROS 2 Workspace`
- **Docker Build**: Build the Docker container
- **Run Simulation**: Start the simulation environment
- **Run Robot Controller**: Launch the robot controller
- **Run State Monitor**: Start the state monitoring

## Packages

### ur_control
**Purpose**: Robot control and trajectory execution
- `robot_controller.py`: Main robot control node
- Supports both simulation and real robot modes
- Integrates with MoveIt for motion planning
- Configurable for different UR robot models

### ur_state_monitor
**Purpose**: Real-time robot state monitoring
- `state_monitor.py`: State monitoring node
- Tracks joint positions, velocities, and forces
- Logs robot status and diagnostics
- Provides safety monitoring

## Configuration

### Robot Models
Supported Universal Robots models:
- UR3, UR3e
- UR5, UR5e
- UR10, UR10e
- UR16e

Configure the robot model in `docker-compose.yml`:
```yaml
environment:
  - ROBOT_MODEL=ur5e  # Change to your robot model
```

### Simulation vs Real Robot
Toggle between simulation and real robot mode:
```yaml
environment:
  - SIMULATION_MODE=true   # Set to false for real robot
```

## GUI Support

For GUI applications (RViz2, Gazebo), enable X11 forwarding in `docker-compose.yml`:

```yaml
environment:
  - DISPLAY=${DISPLAY}
  - QT_X11_NO_MITSHM=1
volumes:
  - /tmp/.X11-unix:/tmp/.X11-unix:rw
```

##  Troubleshooting

### Common Issues

1. **Docker Build Failures**
   ```bash
   # Clean Docker cache and rebuild
   docker system prune -a
   docker-compose build --no-cache ur-simulation
   ```

2. **Permission Issues**
   ```bash
   # Fix script permissions
   chmod +x scripts/*.sh
   ```

3. **ROS 2 Package Not Found**
   ```bash
   # Rebuild the workspace
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ```

### Development Tips

- Use VS Code with the Docker extension for easy container management
- Mount your source code as volumes for live development
- Use `colcon build --symlink-install` for faster iteration
- Check logs in the `logs/` directory for debugging

##  Future Expansion

This platform is designed to support:
- **MiR Robot Integration**: Autonomous mobile robots
- **Digital Twin Development**: Real-time simulation mirroring
- **Multi-Robot Coordination**: Fleet management capabilities
- **Advanced AI Integration**: Machine learning for robotics
- **Industrial IoT**: Integration with factory systems

##  Documentation

- [Setup Guide](docs/setup.md)
- [Usage Examples](docs/usage.md)
- [API Reference](docs/api.md)
- [Troubleshooting](docs/troubleshooting.md)

##  Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test with both simulation and real robot (if available)
5. Submit a pull request

## License

[Add your license here]

## Support

For questions and support:
- Create an issue in this repository
- Check the documentation in the `docs/` folder
- Review the ROS 2 and Universal Robots documentation

---

**Built with for the robotics community**

## Features

- **UR Simulation**: Complete Universal Robots simulation environment using ur_sim
- **ROS 2 Control**: Real-time robot control and state monitoring
- **Docker Integration**: Containerized deployment for consistent environments
- **Modular Architecture**: Ready for expansion with MiR robots and digital twins
- **Dual Mode**: Easy switching between simulation and real robot connections

## Project Structure

```
├── docker/                    # Docker configuration files
│   ├── Dockerfile.ros2-ur     # Main ROS 2 + UR simulation container
│   ├── docker-compose.yml     # Multi-container orchestration
│   └── entrypoint.sh          # Container startup script
├── src/                       # ROS 2 source packages
│   ├── ur_control/            # Robot control package
│   ├── ur_interface/          # Robot interface and communication
│   └── ur_state_monitor/      # State monitoring and logging
├── launch/                    # ROS 2 launch files
│   ├── ur_sim_launch.py       # Simulation mode launch
│   ├── ur_real_launch.py      # Real robot mode launch
│   └── combined_launch.py     # Complete system launch
├── config/                    # Configuration files
│   ├── robot_config.yaml     # Robot parameters
│   └── simulation_config.yaml # Simulation settings
├── scripts/                   # Utility scripts
│   ├── setup_environment.sh   # Environment setup
│   └── run_simulation.sh      # Quick simulation start
└── docs/                      # Documentation
    ├── setup.md               # Setup instructions
    └── usage.md               # Usage guide
```

## Quick Start

### Prerequisites

- Ubuntu 22.04 LTS
- Docker and Docker Compose
- ROS 2 Humble (for host development)

### Installation

1. Clone the repository:
```bash
git clone <your-repo-url>
cd fyp_v2
```

2. Build the Docker environment:
```bash
# Build the Docker image (this may take 10-15 minutes on first run)
docker-compose build

# Alternatively, build just the simulation service
docker-compose build ur-simulation
```

3. Start the simulation:
```bash
# Option 1: Using the convenience script
./scripts/run_simulation.sh --docker

# Option 2: Using Docker Compose directly  
docker-compose up ur-simulation
```

### Usage

#### Simulation Mode
```bash
# Start complete simulation environment
docker-compose up ur-simulation

# Launch ROS 2 control nodes
ros2 launch launch/ur_sim_launch.py
```

#### Real Robot Mode
```bash
# Connect to real UR robot
ros2 launch launch/ur_real_launch.py robot_ip:=<ROBOT_IP>
```

## Development

### Building Packages
```bash
# Build all ROS 2 packages
colcon build --packages-select ur_control ur_interface ur_state_monitor

# Source the workspace
source install/setup.bash
```

### Testing
```bash
# Run unit tests
colcon test

# Run integration tests with simulation
./scripts/test_simulation.sh
```

## Future Roadmap

- [ ] MiR robot base integration
- [ ] Digital twin visualization platform
- [ ] Multi-robot coordination
- [ ] Advanced path planning
- [ ] Real-time monitoring dashboard

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.
