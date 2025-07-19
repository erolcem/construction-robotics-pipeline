#!/bin/bash

# Quick Simulation Startup Script
# This script provides a simple way to start the UR robot simulation

set -e

echo "=== Starting Universal Robots Simulation ==="

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Default parameters
ROBOT_MODEL="ur5e"
USE_GUI="true"
USE_MOVEIT="true"
USE_DOCKER="false"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --robot-model)
            ROBOT_MODEL="$2"
            shift 2
            ;;
        --no-gui)
            USE_GUI="false"
            shift
            ;;
        --no-moveit)
            USE_MOVEIT="false"
            shift
            ;;
        --docker)
            USE_DOCKER="true"
            shift
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --robot-model MODEL    Robot model (ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e, ur20)"
            echo "  --no-gui               Disable GUI (RViz)"
            echo "  --no-moveit            Disable MoveIt motion planning"
            echo "  --docker               Use Docker container"
            echo "  --help                 Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                                    # Start with default settings (UR5e, GUI, MoveIt)"
            echo "  $0 --robot-model ur10e               # Start with UR10e robot"
            echo "  $0 --no-gui --no-moveit             # Start minimal simulation"
            echo "  $0 --docker                         # Start using Docker"
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

print_status "Configuration:"
print_status "  Robot model: $ROBOT_MODEL"
print_status "  GUI enabled: $USE_GUI"
print_status "  MoveIt enabled: $USE_MOVEIT"
print_status "  Use Docker: $USE_DOCKER"

# Check if xauth is available for GUI
if [ "$USE_GUI" = "true" ] && [ "$USE_DOCKER" = "true" ]; then
    if ! command -v xauth &> /dev/null; then
        print_warning "xauth not found. GUI applications may not work in Docker."
        print_warning "Install with: sudo apt-get install xauth"
    fi
    
    # Allow X11 forwarding
    xhost +local: &> /dev/null || true
fi

if [ "$USE_DOCKER" = "true" ]; then
    print_status "Starting Docker simulation..."
    
    # Check if Docker is running
    if ! docker info &> /dev/null; then
        print_error "Docker is not running. Please start Docker and try again."
        exit 1
    fi
    
    # Check if docker-compose is available
    if ! command -v docker-compose &> /dev/null; then
        print_error "docker-compose not found. Please install docker-compose."
        exit 1
    fi
    
    # Set environment variables for docker-compose
    export ROBOT_MODEL="$ROBOT_MODEL"
    export DISPLAY="${DISPLAY}"
    
    # Build and start the container
    print_status "Building Docker image (this may take a while on first run)..."
    docker-compose build ur-simulation
    
    print_status "Starting simulation container..."
    docker-compose up ur-simulation
    
else
    print_status "Starting local simulation..."
    
    # Check if ROS 2 is sourced
    if [ -z "$ROS_DISTRO" ]; then
        print_status "Sourcing ROS 2 environment..."
        source /opt/ros/humble/setup.bash
    fi
    
    # Check if workspace is built
    WORKSPACE_DIR="$(pwd)"
    if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
        print_status "Sourcing workspace..."
        source "$WORKSPACE_DIR/install/setup.bash"
    else
        print_warning "Workspace not built. Building now..."
        "$WORKSPACE_DIR/scripts/setup_environment.sh"
        source "$WORKSPACE_DIR/install/setup.bash"
    fi
    
    # Check if launch file exists
    LAUNCH_FILE="$WORKSPACE_DIR/launch/ur_sim_launch.py"
    if [ ! -f "$LAUNCH_FILE" ]; then
        print_error "Launch file not found: $LAUNCH_FILE"
        exit 1
    fi
    
    # Start the simulation
    print_status "Launching simulation..."
    ros2 launch "$LAUNCH_FILE" \
        robot_model:="$ROBOT_MODEL" \
        use_gui:="$USE_GUI" \
        use_moveit:="$USE_MOVEIT"
fi

print_status "Simulation startup complete!"
