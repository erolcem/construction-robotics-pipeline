# Universal Robots ROS 2 Control and Simulation Platform

A comprehensive ROS 2 workspace for controlling and monitoring Universal Robots in both simulation and real robot environments. This platform provides a Docker-based development environment with full Universal Robots integration.

## 🚀 Features

- **Complete ROS 2 Humble Environment** with Universal Robots packages
- **Docker Containerization** for consistent development across platforms
- **Robot Control Package** (`ur_control`) for trajectory execution and motion planning
- **State Monitoring Package** (`ur_state_monitor`) for real-time robot state tracking
- **Simulation Support** with Gazebo and RViz2
- **Real Robot Integration** ready for Universal Robots (UR3, UR5, UR5e, UR10, UR16)
- **MoveIt Integration** for advanced motion planning
- **Configurable Environment** for different robot models

## 📋 Prerequisites

- Docker and Docker Compose
- VS Code (recommended for development)
- Ubuntu 22.04 or compatible Linux distribution
- At least 8GB RAM and 20GB free disk space

## 🏗️ Project Structure

```
fyp_v2/
├── src/
│   ├── ur_control/           # Robot control package
│   │   ├── ur_control/
│   │   │   ├── __init__.py
│   │   │   └── robot_controller.py
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── ur_state_monitor/     # State monitoring package
│       ├── ur_state_monitor/
│       │   ├── __init__.py
│       │   └── state_monitor.py
│       ├── CMakeLists.txt
│       └── package.xml
├── launch/                   # Launch files for simulation and real robot
│   ├── ur_sim_launch.py
│   └── ur_real_launch.py
├── config/                   # Configuration files
│   ├── robot_config.yaml
│   └── simulation_config.yaml
├── docker/                   # Docker configuration
│   ├── Dockerfile.ros2-ur
│   ├── Dockerfile.minimal
│   └── entrypoint.sh
├── scripts/                  # Utility scripts
│   ├── run_simulation.sh
│   └── setup_environment.sh
├── docs/                     # Documentation
├── logs/                     # Log files
├── docker-compose.yml        # Container orchestration
└── README.md
```

## 🚀 Quick Start

### 1. Clone and Setup

```bash
# Clone this repository
git clone <your-repository-url>
cd fyp_v2

# Make scripts executable
chmod +x scripts/*.sh
```

### 2. Build the Docker Environment

```bash
# Build the ROS 2 container with Universal Robots packages
docker-compose build ur-simulation
```

### 3. Run the Simulation Environment

```bash
# Start the simulation container
docker-compose up ur-simulation
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

## 🎯 VS Code Tasks

The project includes pre-configured VS Code tasks for common operations:

- **Build ROS 2 Workspace**: `Ctrl+Shift+P` → `Tasks: Run Task` → `Build ROS 2 Workspace`
- **Docker Build**: Build the Docker container
- **Run Simulation**: Start the simulation environment
- **Run Robot Controller**: Launch the robot controller
- **Run State Monitor**: Start the state monitoring

## 📦 Packages

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

## 🔧 Configuration

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

## 🖥️ GUI Support

For GUI applications (RViz2, Gazebo), enable X11 forwarding in `docker-compose.yml`:

```yaml
environment:
  - DISPLAY=${DISPLAY}
  - QT_X11_NO_MITSHM=1
volumes:
  - /tmp/.X11-unix:/tmp/.X11-unix:rw
```

## 🔍 Troubleshooting

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

## 🔮 Future Expansion

This platform is designed to support:
- **MiR Robot Integration**: Autonomous mobile robots
- **Digital Twin Development**: Real-time simulation mirroring
- **Multi-Robot Coordination**: Fleet management capabilities
- **Advanced AI Integration**: Machine learning for robotics
- **Industrial IoT**: Integration with factory systems

## 📚 Documentation

- [Setup Guide](docs/setup.md)
- [Usage Examples](docs/usage.md)
- [API Reference](docs/api.md)
- [Troubleshooting](docs/troubleshooting.md)

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test with both simulation and real robot (if available)
5. Submit a pull request

## 📄 License

[Add your license here]

## 📞 Support

For questions and support:
- Create an issue in this repository
- Check the documentation in the `docs/` folder
- Review the ROS 2 and Universal Robots documentation

---

**Built with ❤️ for the robotics community**

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
