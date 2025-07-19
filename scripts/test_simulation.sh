#!/bin/bash

echo "🤖 Starting Universal Robots Simulation..."
echo "This will launch:"
echo "  - UR5e robot in simulation"
echo "  - RViz2 for visualization"
echo "  - Robot controller"
echo "  - State monitor"
echo ""

# Check if we're in a container
if [ -f /.dockerenv ]; then
    echo "✅ Running inside Docker container"
    
    # Source ROS 2 environment
    source /opt/ros/humble/setup.bash
    source /workspace/install/setup.bash
    
    echo "🚀 Launching UR simulation with RViz..."
    
    # For now, let's test with the basic UR bringup
    echo "Starting UR simulation (this may take a moment)..."
    
    # Try to launch the UR simulation
    if command -v ros2 &> /dev/null; then
        echo "ROS 2 found, starting simulation..."
        
        echo "Starting UR5e simulation (this may take a moment)..."
        
        # Start the UR simulation in the background using the correct launch file
        ros2 launch ur_bringup ur5e.launch.py use_fake_hardware:=true launch_rviz:=true &
        UR_PID=$!
        
        # Wait a bit for the simulation to start
        sleep 5
        
        echo "Starting custom control nodes..."
        
        # Start our custom nodes
        ros2 run ur_control robot_controller.py &
        CONTROL_PID=$!
        
        ros2 run ur_state_monitor state_monitor.py &
        MONITOR_PID=$!
        
        echo ""
        echo "🎉 Simulation started successfully!"
        echo "   - UR simulation PID: $UR_PID"
        echo "   - Robot controller PID: $CONTROL_PID"
        echo "   - State monitor PID: $MONITOR_PID"
        echo ""
        echo "💡 If running with GUI support, you should see:"
        echo "   - RViz2 window with the robot model"
        echo "   - Robot arm visualization"
        echo ""
        echo "Press Ctrl+C to stop all processes..."
        
        # Wait for user interrupt
        wait
        
    else
        echo "❌ ROS 2 not found. Make sure you're in the Docker container."
        exit 1
    fi
    
else
    echo "❌ Not running in Docker container."
    echo "Please run this script inside the Docker container:"
    echo ""
    echo "  docker run --rm -it fyp_v2_ur-simulation:latest /workspace/scripts/test_simulation.sh"
    echo ""
    echo "Or for GUI support (Linux with X11):"
    echo "  docker run --rm -it -e DISPLAY=\$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix fyp_v2_ur-simulation:latest /workspace/scripts/test_simulation.sh"
    exit 1
fi
