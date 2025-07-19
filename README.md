# Universal Robots ROS 2 Control & Simulation Platform

A comprehensive ROS 2-based platform for controlling and simulating Universal Robots with Docker containerization, designed for seamless switching between simulation and real robot connections.

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
docker-compose build
```

3. Start the simulation:
```bash
./scripts/run_simulation.sh
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
