#!/bin/bash

# Universal Robots ROS 2 Environment Setup Script
# This script sets up the development environment for UR robot control

set -e  # Exit on any error

echo "=== Universal Robots ROS 2 Environment Setup ==="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running inside Docker
if [ -f /.dockerenv ]; then
    print_status "Running inside Docker container"
    IN_DOCKER=true
else
    print_status "Running on host system"
    IN_DOCKER=false
fi

# Set workspace directory
WORKSPACE_DIR="/workspace"
if [ "$IN_DOCKER" = false ]; then
    WORKSPACE_DIR="$(pwd)"
fi

print_status "Workspace directory: $WORKSPACE_DIR"

# Source ROS 2 environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    print_status "Sourcing ROS 2 Humble environment"
    source /opt/ros/humble/setup.bash
else
    print_error "ROS 2 Humble not found. Please install ROS 2 Humble first."
    exit 1
fi

# Create necessary directories
print_status "Creating necessary directories"
mkdir -p "$WORKSPACE_DIR/logs"
mkdir -p "$WORKSPACE_DIR/build"
mkdir -p "$WORKSPACE_DIR/install"

# Set up Python path for custom packages
export PYTHONPATH="$WORKSPACE_DIR/src:$PYTHONPATH"

# Initialize rosdep if not already done
if [ "$IN_DOCKER" = false ]; then
    if ! rosdep --version &> /dev/null; then
        print_status "Installing rosdep"
        sudo apt-get update
        sudo apt-get install -y python3-rosdep
    fi
    
    if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
        print_status "Initializing rosdep"
        sudo rosdep init
    fi
    
    print_status "Updating rosdep"
    rosdep update
fi

# Install dependencies
if [ -d "$WORKSPACE_DIR/src" ]; then
    print_status "Installing package dependencies"
    cd "$WORKSPACE_DIR"
    rosdep install --from-paths src --ignore-src -r -y
fi

# Build the workspace
if [ -d "$WORKSPACE_DIR/src" ]; then
    print_status "Building ROS 2 workspace"
    cd "$WORKSPACE_DIR"
    
    # Clean previous build if requested
    if [ "$1" = "--clean" ]; then
        print_status "Cleaning previous build"
        rm -rf build install log
    fi
    
    # Build with colcon
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    
    if [ $? -eq 0 ]; then
        print_status "Build successful"
        
        # Source the workspace
        if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
            source "$WORKSPACE_DIR/install/setup.bash"
            print_status "Workspace sourced successfully"
        fi
    else
        print_error "Build failed"
        exit 1
    fi
else
    print_warning "No src directory found, skipping build"
fi

# Set up environment variables
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

# Display for GUI applications
if [ "$IN_DOCKER" = true ]; then
    export DISPLAY=${DISPLAY:-:0}
    export QT_X11_NO_MITSHM=1
fi

# Check if UR packages are available
if ros2 pkg list | grep -q "ur_robot_driver"; then
    print_status "UR robot driver packages found"
else
    print_warning "UR robot driver packages not found"
    if [ "$IN_DOCKER" = false ]; then
        print_status "Installing UR packages"
        sudo apt-get update
        sudo apt-get install -y ros-humble-ur-robot-driver ros-humble-ur-moveit-config
    fi
fi

# Check MoveIt installation
if ros2 pkg list | grep -q "moveit"; then
    print_status "MoveIt packages found"
else
    print_warning "MoveIt packages not found"
    if [ "$IN_DOCKER" = false ]; then
        print_status "Installing MoveIt"
        sudo apt-get install -y ros-humble-moveit
    fi
fi

# Create alias for common commands
if [ "$IN_DOCKER" = false ]; then
    echo "# UR ROS 2 aliases" >> ~/.bashrc
    echo "alias ur_sim='ros2 launch launch/ur_sim_launch.py'" >> ~/.bashrc
    echo "alias ur_real='ros2 launch launch/ur_real_launch.py'" >> ~/.bashrc
    echo "alias ur_monitor='ros2 run ur_state_monitor state_monitor.py'" >> ~/.bashrc
fi

print_status "Environment setup complete!"
print_status ""
print_status "Available commands:"
print_status "  ros2 launch launch/ur_sim_launch.py    - Start simulation"
print_status "  ros2 launch launch/ur_real_launch.py   - Connect to real robot"
print_status "  ros2 run ur_state_monitor state_monitor.py - Start state monitoring"
print_status ""
print_status "To run with Docker:"
print_status "  docker-compose up ur-simulation        - Start simulation container"
print_status "  docker-compose --profile real-robot up - Start real robot container"
