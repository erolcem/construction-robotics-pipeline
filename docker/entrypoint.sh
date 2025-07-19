#!/bin/bash
set -e

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Source workspace if it exists
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

# Set display for GUI applications (if available)
export QT_X11_NO_MITSHM=1

# Check if we should run in headless mode
if [ "${HEADLESS}" = "true" ]; then
    echo "Running in headless mode..."
    export DISPLAY=""
fi

# Default robot model
export ROBOT_MODEL=${ROBOT_MODEL:-ur5e}
export SIMULATION_MODE=${SIMULATION_MODE:-true}

echo "================================================"
echo "Universal Robots ROS 2 Environment"
echo "Robot Model: $ROBOT_MODEL"
echo "Simulation Mode: $SIMULATION_MODE"
echo "Headless: ${HEADLESS:-false}"
echo "================================================"

# If no command is provided, start interactive bash
if [ $# -eq 0 ]; then
    echo "Starting interactive bash session..."
    echo "Available commands:"
    echo "  - Launch simple simulation: ros2 launch launch/ur_simple_sim.launch.py"
    echo "  - Run robot controller: ros2 run ur_control robot_controller.py"
    echo "  - Run state monitor: ros2 run ur_state_monitor state_monitor.py"
    echo "  - List UR packages: ros2 pkg list | grep ur"
    exec bash
else
    # Execute provided command
    exec "$@"
fi
