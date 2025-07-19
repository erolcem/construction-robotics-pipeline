#!/bin/bash

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Check if running in a container or headless mode
HEADLESS=${HEADLESS:-false}
if [ "$HEADLESS" = "true" ] || [ -n "$CONTAINER_ENV" ] || [ ! -z "$DOCKER_ENV" ]; then
    echo "Running in headless mode..."
    export DISPLAY=""
    RVIZ_ARG=""
else
    RVIZ_ARG="launch_rviz:=true"
fi

echo "================================================"
echo "Universal Robots ROS 2 Environment"
echo "Robot Model: ur5e"
echo "Simulation Mode: true"
echo "Headless: $HEADLESS"
echo "================================================"

echo -e "${BLUE}🤖 Starting Universal Robots Simulation...${NC}"
echo "This will launch:"
echo "  - UR5e robot in simulation"
echo "  - RViz2 for visualization"
echo "  - Robot controller"
echo "  - State monitor"
echo ""

# Check if we're in a container
if [ -f /.dockerenv ]; then
    echo -e "${GREEN}✅ Running inside Docker container${NC}"
fi

# Setup ROS environment
source /opt/ros/humble/setup.bash
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

echo -e "${YELLOW}🚀 Launching UR simulation with RViz...${NC}"
echo "Starting UR simulation (this may take a moment)..."

# Detect ROS 2 installation
if [ -d "/opt/ros/humble" ]; then
    echo "ROS 2 found, starting simulation..."
    
    echo "Starting UR5e simulation (this may take a moment)..."
    
    # Launch UR simulation with fake hardware
    ros2 launch /workspace/launch/ur_simulation.launch.py \
        ur_type:=ur5e \
        use_fake_hardware:=true \
        fake_sensor_commands:=true \
        $RVIZ_ARG &
    
    SIMULATION_PID=$!
    
    # Wait a bit for the simulation to start
    sleep 5
    
    echo "Starting custom control nodes..."
    
    # Launch robot controller
    ros2 run ur_control robot_controller.py &
    CONTROLLER_PID=$!
    
    # Launch state monitor  
    ros2 run ur_state_monitor state_monitor.py &
    MONITOR_PID=$!
    
    echo ""
    echo -e "${GREEN}🎉 Simulation started successfully!${NC}"
    echo "   - UR simulation PID: $SIMULATION_PID"
    echo "   - Robot controller PID: $CONTROLLER_PID"
    echo "   - State monitor PID: $MONITOR_PID"
    echo ""
    echo -e "${BLUE}💡 If running with GUI support, you should see:${NC}"
    echo "   - RViz2 window with the robot model"
    echo "   - Robot arm visualization"
    echo ""
    echo "Press Ctrl+C to stop all processes..."
    
    # Function to cleanup background processes
    cleanup() {
        echo ""
        echo -e "${YELLOW}🛑 Stopping simulation...${NC}"
        kill $SIMULATION_PID $CONTROLLER_PID $MONITOR_PID 2>/dev/null || true
        wait 2>/dev/null || true
        echo -e "${GREEN}✅ All processes stopped${NC}"
        exit 0
    }
    
    # Set up signal handler
    trap cleanup SIGINT SIGTERM
    
    # Wait for all background processes
    wait $SIMULATION_PID $CONTROLLER_PID $MONITOR_PID
    
else
    echo -e "${RED}❌ ROS 2 Humble not found. Please install ROS 2 Humble first.${NC}"
    exit 1
fi
