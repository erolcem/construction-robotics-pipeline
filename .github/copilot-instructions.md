<!-- Use this file to provide workspace-specific custom instructions to Copilot. For more details, visit https://code.visualstudio.com/docs/copilot/copilot-customization#_use-a-githubcopilotinstructionsmd-file -->

# ROS 2 Universal Robots Project Instructions

This workspace contains a ROS 2 project for controlling and simulating Universal Robots (UR) robotic arms. When working with this codebase, please follow these guidelines:

## Project Context
- **Framework**: ROS 2 Humble
- **Robot Platform**: Universal Robots (UR3, UR3e, UR5, UR5e, UR10, UR10e, UR16e, UR20)
- **Simulation**: Gazebo with ur_sim
- **Motion Planning**: MoveIt 2
- **Containerization**: Docker with docker-compose
- **Programming Languages**: Python 3 for ROS nodes, YAML for configuration

## Code Style Guidelines
1. **Python**: Follow PEP 8 standards with 4-space indentation
2. **ROS 2 Nodes**: Use rclpy and follow ROS 2 Python conventions
3. **File Naming**: Use snake_case for Python files and kebab-case for configuration files
4. **Documentation**: Include docstrings for all classes and functions

## ROS 2 Specific Guidelines
- Always use `self.get_logger()` for logging in ROS nodes
- Include proper parameter declarations with `declare_parameter()`
- Use appropriate QoS profiles for publishers and subscribers
- Handle node lifecycle properly with try/except blocks
- Include proper cleanup in node destructors

## Docker Integration
- All ROS 2 development should be containerized
- Use the provided Dockerfile and docker-compose.yml
- Mount source code as volumes for development
- Ensure proper X11 forwarding for GUI applications

## Package Structure
- `src/ur_control/`: Robot control logic and trajectory execution
- `src/ur_state_monitor/`: State monitoring and logging
- `launch/`: Launch files for different modes (simulation vs real robot)
- `config/`: YAML configuration files
- `docker/`: Docker configuration and compose files

## Safety Considerations
- Always include safety checks for joint limits and velocities
- Implement emergency stop functionality
- Validate all trajectory points before execution
- Include timeout mechanisms for robot communication

## Future Expansion
This project is designed to expand with:
- MiR robot base integration
- Digital twin visualization
- Multi-robot coordination
- Advanced path planning algorithms

When generating code, prioritize:
1. Safety and error handling
2. Modularity and reusability
3. Clear documentation and comments
4. ROS 2 best practices
5. Docker compatibility
